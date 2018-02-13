// Copyright 2017-2018 Leland Lucius
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/lock.h>

#include "esp_err.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern "C"
{
#include "redposix.h"
#include "redosserv.h"
#include "redutils.h"
#include "redmacs.h"
#include "redvolume.h"
#include "redpath.h"

// include redconf.c so it can be found in the "include" paths
#include "redconf.c"
}

#include "redflash.h"

#define FREE_FD -1

static const char *TAG = "redflash";

static RedFlash *rf_instances[REDCONF_VOLUME_COUNT];
static _lock_t lock;

RedFlash::RedFlash()
{
    fds = NULL;
    mounted = false;
    registered = false;

    _lock_init(&lock);
}

RedFlash::~RedFlash()
{
    term();

    _lock_close(&lock);
}

esp_err_t RedFlash::init(const red_flash_config_t *config)
{
    ESP_LOGD(TAG, "%s", __func__);

    cfg = *config;
    label_len = strlen(cfg.volume_label);

    if (cfg.flash)
    {
        sector_sz = cfg.flash->sector_size();
        chip_sz = cfg.flash->chip_size();
    }
    else
    {
        part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                        ESP_PARTITION_SUBTYPE_ANY,
                                        cfg.part_label);
        if (part == NULL)
        {
            ESP_LOGE(TAG, "Partition '%s' not found", cfg.part_label);
            return ESP_ERR_NOT_FOUND;
        }

        sector_sz = SPI_FLASH_SEC_SIZE;
        chip_sz = part->size;
    }
    block_cnt = chip_sz / sector_sz;

    wl_config_t wl_cfg =
    {
        .start_addr = 0,
        .full_mem_size = chip_sz,
        .page_size = sector_sz,
        .sector_size = sector_sz,
        .updaterate = 16,
        .wr_size = 16,
        .version = 0,
        .temp_buff_size = 32,
        .crc = 0
    };

    flash = new WL_Flash();
    if (flash == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t esperr = flash->config(&wl_cfg, this);
    if (esperr != ESP_OK)
    {
        return esperr;
    }

    esperr = flash->init();
    if (esperr != ESP_OK)
    {
        return esperr;
    }

    _lock_acquire(&lock);

    int err = red_init();
    if (err == -1)
    {
        return ESP_FAIL;
    }

    uint8_t volnum;
    err = RedPathVolumeLookup(cfg.volume_label, &volnum);
    if (err != 0)
    {
        return ESP_FAIL;
    }
    rf_instances[volnum] = this;

    err = red_mount(cfg.volume_label);

#if (REDCONF_READ_ONLY == 0) && (REDCONF_API_POSIX_FORMAT == 1)
    if (err == -1 && red_errno == RED_EIO)
    {
        if (red_errno == RED_EIO && !cfg.auto_format)
        {
            return ESP_FAIL;
        }

        err = red_format(cfg.volume_label);
        if (err == -1)
        {
            return ESP_FAIL;
        }

        err = red_mount(cfg.volume_label);
    }
#endif // (REDCONF_READ_ONLY == 0) && (REDCONF_API_POSIX_FORMAT == 1)

    if (err == -1)
    {
        return ESP_FAIL;
    }

    mounted = true;

    fds = new vfs_fd_t[cfg.open_files];
    if (fds == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    for (int i = 0; i < cfg.open_files; i++)
    {
        fds[i].red_fd = FREE_FD;
    }

    esp_vfs_t vfs = {};

    vfs.flags = ESP_VFS_FLAG_CONTEXT_PTR;

    vfs.open_p = &open_p;
    vfs.close_p = &close_p;
    vfs.read_p = &read_p;
    vfs.lseek_p = &lseek_p;
    vfs.fstat_p = &fstat_p;
    vfs.stat_p = &stat_p;

#if REDCONF_API_POSIX_READDIR == 1
    vfs.opendir_p = &opendir_p;
    vfs.readdir_p = &readdir_p;
    vfs.readdir_r_p = &readdir_r_p;
    vfs.telldir_p = &telldir_p;
    vfs.seekdir_p = &seekdir_p;
    vfs.closedir_p = &closedir_p;
#endif // REDCONF_API_POSIX_READDIR == 1

#if REDCONF_READ_ONLY == 0
    vfs.write_p = &write_p;
    vfs.fsync_p = &fsync_p;

#if REDCONF_API_POSIX_LINK == 1
    vfs.link_p = &link_p;
#endif // REDCONF_API_POSIX_LINK == 1

#if REDCONF_API_POSIX_UNLINK == 1
    vfs.unlink_p = &unlink_p;
#endif // REDCONF_API_POSIX_UNLINK == 1

#if REDCONF_API_POSIX_RENAME == 1
    vfs.rename_p = &rename_p;
#endif // REDCONF_API_POSIX_RENAME == 1

#if REDCONF_API_POSIX_MKDIR == 1
    vfs.mkdir_p = &mkdir_p;
#endif // REDCONF_API_POSIX_MKDIR == 1

#if REDCONF_API_POSIX_RMDIR == 1
    vfs.rmdir_p = &rmdir_p;
#endif // REDCONF_API_POSIX_RMDIR == 1
#endif // REDCONF_READ_ONLY == 0

    esperr = esp_vfs_register(cfg.base_path, &vfs, this);
    if (esperr != ESP_OK)
    {
        return err;
    }

    registered = true;

    _lock_release(&lock);

    return ESP_OK;
}

void RedFlash::term()
{
    ESP_LOGD(TAG, "%s", __func__);

    _lock_acquire(&lock);

    if (registered)
    {
        for (int i = 0; i < cfg.open_files; i++)
        {
            if (fds[i].red_fd != FREE_FD)
            {
                red_close(fds[i].red_fd);
            }
        }

        esp_vfs_unregister(cfg.base_path);
        registered = false;
    }

    if (fds)
    {
        delete [] fds;
        fds = NULL;
    }

    if (mounted)
    {
        red_umount(cfg.volume_label);
        mounted = false;
    }

    red_uninit();

    _lock_release(&lock);
}

// ============================================================================
// ESP32 VFS implementation
// ============================================================================

int RedFlash::map_red_errno(int err)
{
    if (err >= 0)
    {
        return err;
    }

    switch (red_errno)
    {
        case RED_ENAMETOOLONG:
            errno = ENAMETOOLONG;
        break;
        case RED_ENOSYS:
            errno = ENOSYS;
        break;
        case RED_ENOTEMPTY:
            errno = ENOTEMPTY;
        break;
        case RED_EUSERS:
            errno = 131;
        break;
        default:
            errno = red_errno;
        break;
    }

    return -1;
}

int RedFlash::get_free_fd()
{
    for (int i = 0; i < cfg.open_files; i++)
    {
        if (fds[i].red_fd == FREE_FD)
        {
            return i;
        }
    }

    return FREE_FD;
}

char *RedFlash::make_red_path(const char *path)
{
    int len = strlen(path);
    char *newpath = (char *) malloc(len + label_len + 1);
    if (newpath)
    {
        memcpy(newpath, cfg.volume_label, label_len);
        memcpy(&newpath[label_len], path, len + 1); // copy null terminator too
    }
    else
    {
        red_errno = RED_ENOMEM;
    }

    return newpath;
}
off_t RedFlash::lseek_p(void *ctx, int fd, off_t size, int mode)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs;

    REDWHENCE red_mode;
    if (mode == SEEK_SET)
    {
        red_mode = RED_SEEK_SET;
    }
    else if (mode == SEEK_CUR)
    {
        red_mode = RED_SEEK_CUR;
    }
    else if (mode == SEEK_END)
    {
        red_mode = RED_SEEK_END;
    }
    else
    {
        errno = EINVAL;
        return -1;
    }

    _lock_acquire(&lock);

    fd = that->fds[fd].red_fd;
    if (fd == FREE_FD)
    {
        red_errno = RED_EBADF;
        rs = -1;
    }
    else
    {
        rs = red_lseek(fd, size, red_mode);
    }

    _lock_release(&lock);

    return map_red_errno(rs);
}

ssize_t RedFlash::read_p(void *ctx, int fd, void *dst, size_t size)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs;

    _lock_acquire(&lock);

    fd = that->fds[fd].red_fd;
    if (fd == FREE_FD)
    {
        red_errno = RED_EBADF;
        rs = -1;
    }
    else
    {
        rs = red_read(fd, dst, size);
    }

    _lock_release(&lock);

    return map_red_errno(rs);
}

int RedFlash::open_p(void *ctx, const char *path, int flags, int mode)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs = -1;

    int red_flags = 0;
    if ((flags & O_ACCMODE) == O_RDONLY)
    {
        red_flags = RED_O_RDONLY;
    }
    else if ((flags & O_ACCMODE) == O_WRONLY)
    {
        red_flags = RED_O_WRONLY;
    }
    else if ((flags & O_ACCMODE) == O_RDWR)
    {
        red_flags = RED_O_RDWR;
    }

    if (flags & O_CREAT)
    {
        red_flags |= RED_O_CREAT;
    }

    if (flags & O_EXCL)
    {
        red_flags |= RED_O_EXCL;
    }

    if (flags & O_TRUNC)
    {
        red_flags |= RED_O_TRUNC;
    }
    
    if (flags & O_APPEND)
    {
        red_flags |= RED_O_APPEND;
    }

    char *rpath = that->make_red_path(path);
    if (rpath)
    {
        _lock_acquire(&lock);

        int fd = that->get_free_fd();
        if (fd == FREE_FD)
        {
            red_errno = RED_ENFILE;
        }
        else
        {
            rs = red_open(rpath, red_flags);
            if (rs >= 0)
            {
                that->fds[fd].red_fd = rs;
                rs = fd;
            }
        }

        _lock_release(&lock);

        free(rpath);
    }

    return map_red_errno(rs);
}

int RedFlash::close_p(void *ctx, int fd)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs;

    _lock_acquire(&lock);

    int red_fd = that->fds[fd].red_fd;
    if (red_fd == FREE_FD)
    {
        red_errno = RED_EBADF;
        rs = -1;
    }
    else
    {
        rs = red_close(red_fd);
        that->fds[fd].red_fd = FREE_FD;
    }

    _lock_release(&lock);

    return map_red_errno(rs);
}

int RedFlash::fstat_p(void *ctx, int fd, struct stat *st)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs;

    _lock_acquire(&lock);

    fd = that->fds[fd].red_fd;
    if (fd == FREE_FD)
    {
        red_errno = RED_EBADF;
        rs = -1;
    }
    else
    {
        REDSTAT stat;
        rs = red_fstat(fd, &stat);
        if (rs == 0)
        {
            *st = {};
            st->st_dev = stat.st_dev;
            st->st_ino = stat.st_ino;
            st->st_mode = stat.st_mode;
            st->st_nlink = stat.st_nlink;
            st->st_size = stat.st_size;

#if REDCONF_INODE_TIMESTAMPS == 1
            st->st_atime = stat.st_atime;
            st->st_mtime = stat.st_mtime;
            st->st_ctime = stat.st_ctime;
#endif
#if REDCONF_INODE_BLOCKS == 1
            st->st_blocks = stat.st_blocks;
#endif

            if (RED_S_ISDIR(stat.st_mode))
            {
                st->st_mode = S_IFDIR | S_IRWXU | S_IRWXG | S_IRWXO;
            }
            else
            {
                st->st_mode = S_IFREG | S_IRWXU | S_IRWXG | S_IRWXO;
            }
        }
    }

    _lock_release(&lock);

    return map_red_errno(rs);
}

int RedFlash::stat_p(void *ctx, const char *path, struct stat *st)
{
    int32_t rs;

    int32_t fd = open_p(ctx, path, O_RDONLY, 0);
    if (fd == -1)
    {
        return map_red_errno(fd);
    }

    rs = fstat_p(ctx, fd, st);

    REDSTATUS err = red_errno;
    close_p(ctx, fd);
    red_errno = err;

    return map_red_errno(rs);
}

#if REDCONF_API_POSIX_READDIR == 1

typedef struct
{
    DIR dir;                // must be first...ESP32 VFS expects it...
    struct dirent dirent;
    REDDIR *red_dir;
    long off;
} vfs_red_dir_t;

DIR *RedFlash::opendir_p(void *ctx, const char *name)
{
    RedFlash *that = (RedFlash *) ctx;

    vfs_red_dir_t *vfs_dir = (vfs_red_dir_t *) malloc(sizeof(vfs_red_dir_t));
    if (vfs_dir == NULL)
    {
        errno = ENOMEM;
        return NULL;
    }
    *vfs_dir = {};

    char *rname = that->make_red_path(name);
    if (rname)
    {
        _lock_acquire(&lock);

        vfs_dir->red_dir = red_opendir(rname);

        _lock_release(&lock);

        free(rname);
    }

    if (rname == NULL || vfs_dir->red_dir == NULL)
    {
        free(vfs_dir);
        vfs_dir = NULL;
        map_red_errno(-1);
    }

    return (DIR *) vfs_dir;
}

struct dirent *RedFlash::readdir_p(void *ctx, DIR *pdir)
{
    vfs_red_dir_t *vfs_dir = (vfs_red_dir_t *) pdir;
    if (vfs_dir == NULL)
    {
        errno = EBADF;
        return NULL;
    }

    struct dirent *out_dirent = NULL;

    int err = readdir_r_p(ctx, pdir, &vfs_dir->dirent, &out_dirent);
    if (err != 0)
    {
        errno = err;
    }

    return out_dirent;
}

int RedFlash::readdir_r_p(void *ctx, DIR *pdir, struct dirent *entry, struct dirent **out_dirent)
{
    vfs_red_dir_t *vfs_dir = (vfs_red_dir_t *) pdir;
    if (vfs_dir == NULL)
    {
        errno = EBADF;
        return errno;
    }

    _lock_acquire(&lock);

    red_errno = 0;
    REDDIRENT *de = red_readdir(vfs_dir->red_dir);

    _lock_release(&lock);

    if (de == NULL)
    {
        if (red_errno == 0)
        {
            *out_dirent = NULL;
            return 0;
        }

        map_red_errno(-1);
        return errno;        
    }

    *entry = {};
    entry->d_ino = de->d_ino;
    if (RED_S_ISREG(de->d_stat.st_mode))
    {
        entry->d_type = DT_REG;
    }
    else if (RED_S_ISDIR(de->d_stat.st_mode))
    {
        entry->d_type = DT_DIR;
    }
    else
    {
        entry->d_type = DT_UNKNOWN;
    }
    size_t len = strlcpy(entry->d_name, de->d_name, sizeof(entry->d_name));

    // This "shouldn't" happen, but the name length can be customized and may
    // be longer than what's provided in "struct dirent"
    if (len >= sizeof(entry->d_name))
    {
        errno = ENAMETOOLONG;
        return errno;
    }

    vfs_dir->off++;

    *out_dirent = entry;

    return 0;
}

long RedFlash::telldir_p(void *ctx, DIR *pdir)
{
    vfs_red_dir_t *vfs_dir = (vfs_red_dir_t *) pdir;
    if (vfs_dir == NULL)
    {
        errno = EBADF;
        return errno;
    }

    return vfs_dir->off;
}

void RedFlash::seekdir_p(void *ctx, DIR *pdir, long offset)
{
    vfs_red_dir_t *vfs_dir = (vfs_red_dir_t *) pdir;
    if (vfs_dir == NULL)
    {
        errno = EBADF;
        return;
    }

    _lock_acquire(&lock);

    // ESP32 VFS expects simple 0 to n counted directory offsets but red
    // doesn't so we need to "translate"...
    red_rewinddir(vfs_dir->red_dir);
    for (vfs_dir->off = 0; vfs_dir->off < offset; ++vfs_dir->off)
    {
        red_errno = 0;
        REDDIRENT *de = red_readdir(vfs_dir->red_dir);
        if (de == NULL)
        {
            map_red_errno(-1);
            break;
        }
    }

    _lock_release(&lock);

    return;
}

int RedFlash::closedir_p(void *ctx, DIR *pdir)
{
    int32_t rs;

    vfs_red_dir_t *vfs_dir = (vfs_red_dir_t *) pdir;
    if (vfs_dir == NULL)
    {
        errno = EBADF;
        return -1;
    }

    _lock_acquire(&lock);

    rs = red_closedir(vfs_dir->red_dir);

    _lock_release(&lock);

    free(vfs_dir);

    return map_red_errno(rs);
}

#endif //REDCONF_API_POSIX_READDIR == 1

#if REDCONF_READ_ONLY == 0

ssize_t RedFlash::write_p(void *ctx, int fd, const void *data, size_t size)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs;

    _lock_acquire(&lock);

    fd = that->fds[fd].red_fd;
    if (fd == FREE_FD)
    {
        red_errno = RED_EBADF;
        rs = -1;
    }
    else
    {
        rs = red_write(fd, data, size);
    }

    _lock_release(&lock);

    return map_red_errno(rs);
}

int RedFlash::fsync_p(void *ctx, int fd)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs;

    _lock_acquire(&lock);

    fd = that->fds[fd].red_fd;
    if (fd == FREE_FD)
    {
        red_errno = RED_EBADF;
        rs = -1;
    }
    else
    {
        rs = red_fsync(fd);
    }

    _lock_release(&lock);

    return map_red_errno(rs);
}

#if REDCONF_API_POSIX_LINK == 1
int RedFlash::link_p(void *ctx, const char *n1, const char *n2)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs = -1;

    char *rn1 = that->make_red_path(n1);
    if (rn1)
    {
        char *rn2 = that->make_red_path(n2);
        if (rn2)
        {
            _lock_acquire(&lock);

            rs = red_link(rn1, rn2);

            _lock_release(&lock);

            free(rn2);
        }

        free(rn1);
    }

    return map_red_errno(rs);
}
#endif // REDCONF_API_POSIX_LINK == 1

#if REDCONF_API_POSIX_UNLINK == 1
int RedFlash::unlink_p(void *ctx, const char *path)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs = -1;

    char *rpath = that->make_red_path(path);
    if (rpath)
    {
        _lock_acquire(&lock);

        rs = red_unlink(rpath);

        _lock_release(&lock);

        free(rpath);
    }

    return map_red_errno(rs);
}
#endif // REDCONF_API_POSIX_UNLINK == 1

#if REDCONF_API_POSIX_RENAME == 1
int RedFlash::rename_p(void *ctx, const char *src, const char *dst)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs = -1;

    char *rsrc = that->make_red_path(src);
    if (rsrc)
    {
        char *rdst = that->make_red_path(dst);
        if (rdst)
        {
            _lock_acquire(&lock);

            rs = red_rename(rsrc, rdst);

            _lock_release(&lock);

            free(rdst);
        }

        free(rsrc);
    }

    return map_red_errno(rs);
}
#endif // REDCONF_API_POSIX_RENAME == 1

#if REDCONF_API_POSIX_MKDIR == 1
int RedFlash::mkdir_p(void *ctx, const char *name, mode_t mode)
{
    RedFlash *that = (RedFlash *) ctx;
    int32_t rs = -1;

    char *rname = that->make_red_path(name);
    if (rname)
    {
        _lock_acquire(&lock);

        rs = red_mkdir(rname);

        _lock_release(&lock);

        free(rname);
    }

    return map_red_errno(rs);
}
#endif // REDCONF_API_POSIX_MKDIR == 1

#if REDCONF_API_POSIX_RMDIR == 1
int RedFlash::rmdir_p(void *ctx, const char *name)
{
    RedFlash *that = (RedFlash *) ctx;
    int rs = -1;

    char *rname = that->make_red_path(name);
    if (rname)
    {
        _lock_acquire(&lock);

        rs = red_rmdir(rname);

        _lock_release(&lock);

        free(rname);
    }

    return map_red_errno(rs);
}
#endif // REDCONF_API_POSIX_RMDIR == 1

#endif // REDCONF_READ_ONLY == 0

// ============================================================================
// Flash_Access implementation
// ============================================================================

size_t RedFlash::chip_size()
{
    return chip_sz;
}

esp_err_t RedFlash::erase_sector(size_t sector)
{
    esp_err_t err;

#if REDCONF_READ_ONLY == 0
    if (cfg.flash)
    {
        err = cfg.flash->erase_sector(sector);
    }
    else
    {
        err = esp_partition_erase_range(part, sector * sector_sz, sector_sz);
    }
#else
    err = ESP_FAIL;
#endif

    return err;
}

esp_err_t RedFlash::erase_range(size_t start_address, size_t size)
{
    esp_err_t err;

#if REDCONF_READ_ONLY == 0
    if (cfg.flash)
    {
        err = cfg.flash->erase_range(start_address, size);
    }
    else
    {
        err = esp_partition_erase_range(part, start_address, size);
    }
#else
    err = ESP_FAIL;
#endif

    return err;
}

esp_err_t RedFlash::write(size_t dest_addr, const void *src, size_t size)
{
    esp_err_t err;

#if REDCONF_READ_ONLY == 0
    if (cfg.flash)
    {
        err = cfg.flash->write(dest_addr, src, size);
    }
    else
    {
        err = esp_partition_write(part, dest_addr, src, size);
    }
#else
    err = ESP_FAIL;
#endif

    return err;
}

esp_err_t RedFlash::read(size_t src_addr, void *dest, size_t size)
{
    esp_err_t err;

    if (cfg.flash)
    {
        err = cfg.flash->read(src_addr, dest, size);
    }
    else
    {
        err = esp_partition_read(part, src_addr, dest, size);
    }

    return err;
}

size_t RedFlash::sector_size()
{
    return sector_sz;
}

// ============================================================================
// RedFS interface implementation
// (mostly copied from Red sources
// ============================================================================
extern "C"
{
    static SemaphoreHandle_t xMutex;
#if defined(configSUPPORT_STATIC_ALLOCATION) && (configSUPPORT_STATIC_ALLOCATION == 1)
    static StaticSemaphore_t xMutexBuffer;
#endif

    void RedOsAssertFail(const char *pszFileName, uint32_t ulLineNum)
    {
        printf("Assertion failed in \"%s\" at line %u\n",
               (pszFileName == NULL) ? "" : pszFileName,
               (unsigned) ulLineNum);
        abort();
    }

    REDSTATUS RedOsClockInit(void)
    {
        return 0;
    }

    REDSTATUS RedOsClockUninit(void)
    {
        return 0;
    }

    uint32_t RedOsClockGetTime(void)
    {
        return (uint32_t) time(NULL);
    }

    REDSTATUS RedOsMutexInit(void)
    {
#if defined(configSUPPORT_STATIC_ALLOCATION) && (configSUPPORT_STATIC_ALLOCATION == 1)
        xMutex = xSemaphoreCreateMutexStatic(&xMutexBuffer);
        if (xMutex == NULL)
        {
            /*  The only error case for xSemaphoreCreateMutexStatic is that the mutex
                buffer parameter is NULL, which is not the case.
            */
            REDERROR();
            return -RED_EINVAL;
        }
#else
        xMutex = xSemaphoreCreateMutex();
        if (xMutex == NULL)
        {
            return -RED_ENOMEM;
        }
#endif

        return 0;
    }

    REDSTATUS RedOsMutexUninit(void)
    {
        vSemaphoreDelete(xMutex);
        xMutex = NULL;

        return 0;
    }

    void RedOsMutexAcquire(void)
    {
        while (xSemaphoreTake(xMutex, portMAX_DELAY) != pdTRUE)
        {
        }
    }

    void RedOsMutexRelease(void)
    {
        BaseType_t xSuccess;

        xSuccess = xSemaphoreGive(xMutex);
        REDASSERT(xSuccess == pdTRUE);
    }

    void RedOsOutputString(const char *pszString)
    {
        if (pszString == NULL)
        {
            REDERROR();
        }
        else
        {
            printf("%s", pszString);
        }
    }

    REDSTATUS RedOsBDevOpen(uint8_t bVolNum, BDEVOPENMODE mode)
    {
        if (bVolNum >= REDCONF_VOLUME_COUNT)
        {
            return -RED_EINVAL;
        }

        return 0;
    }

    REDSTATUS RedOsBDevClose(uint8_t bVolNum)
    {
        if (bVolNum >= REDCONF_VOLUME_COUNT)
        {
            return -RED_EINVAL;
        }

        return 0;
    }

    REDSTATUS RedOsBDevRead(uint8_t bVolNum, uint64_t ullSectorStart, uint32_t ulSectorCount, void *pBuffer)
    {
        if ((bVolNum >= REDCONF_VOLUME_COUNT) ||
            (ullSectorStart >= gaRedVolConf[bVolNum].ullSectorCount) ||
            ((gaRedVolConf[bVolNum].ullSectorCount - ullSectorStart) < ulSectorCount) ||
            (pBuffer == NULL))
        {
            return -RED_EINVAL;
        }

        RedFlash *that = rf_instances[bVolNum];
        if (that == NULL)
        {
            return -RED_EINVAL;
        }

        uint32_t addr = ullSectorStart * REDCONF_BLOCK_SIZE;
        size_t size = ulSectorCount * REDCONF_BLOCK_SIZE;
        esp_err_t err;

        err = that->flash->read(addr, pBuffer, size);
        if (err != ESP_OK)
        {
            return -RED_EIO;
        }

        return 0;
    }

    REDSTATUS RedOsBDevWrite(uint8_t bVolNum, uint64_t ullSectorStart, uint32_t ulSectorCount, const void *pBuffer)
    {
#if REDCONF_READ_ONLY == 0
        if ((bVolNum >= REDCONF_VOLUME_COUNT) ||
            (ullSectorStart >= gaRedVolConf[bVolNum].ullSectorCount) ||
            ((gaRedVolConf[bVolNum].ullSectorCount - ullSectorStart) < ulSectorCount) ||
            (pBuffer == NULL))
        {
            return -RED_EINVAL;
        }

        RedFlash *that = rf_instances[bVolNum];
        if (that == NULL)
        {
            return -RED_EINVAL;
        }

        uint32_t addr = ullSectorStart * REDCONF_BLOCK_SIZE;
        size_t size = ulSectorCount * REDCONF_BLOCK_SIZE;
        esp_err_t err;

        err = that->flash->erase_range(addr, size);
        if (err != ESP_OK)
        {
            return -RED_EIO;
        }

        err = that->flash->write(addr, pBuffer, size);
        if (err != ESP_OK)
        {
            return -RED_EIO;
        }

        return 0;
#else
        return -RED_EROFS;
#endif // REDCONF_READ_ONLY == 0
    }

    REDSTATUS RedOsBDevFlush(uint8_t bVolNum)
    {
#if REDCONF_READ_ONLY == 0
        if (bVolNum >= REDCONF_VOLUME_COUNT)
        {
            return -RED_EINVAL;
        }

        return 0;
#else
        return -RED_EROFS;
#endif // REDCONF_READ_ONLY == 0
    }
}

