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

#if !defined(_REDFLASH_H_)
#define _REDFLASH_H_ 1

#include <sys/lock.h>

#include "esp_err.h"
#include "esp_vfs.h"
#include "esp_partition.h"
#include "WL_Flash.h"

#include "extflash.h"

extern "C"
{
#include "redconf.h"
#include "rederrno.h"
#include "redcoreapi.h"

#if REDCONF_API_POSIX != 1
#error REDCONF_API_POSIX must be set to 1
#endif // REDCONF_API_POSIX != 1

#if REDCONF_TASK_COUNT != 1
#error REDCONF_TASK_COUNT must be set to 1
#endif // REDCONF_TASK_COUNT != 1
}

typedef struct
{
    ExtFlash *flash;            // initialized ExtFlash, or NULL for internal flash
    const char *part_label;     // partition label if using internal flash
    const char *volume_label;   // RedFS volume label from gaRedVolConf
    const char *base_path;      // mount point
    int open_files;             // number of open files to support
    bool auto_format;           // true=format if not valid
} red_flash_config_t;

class RedFlash : public Flash_Access
{
public:
    RedFlash();
    virtual ~RedFlash();

    esp_err_t init(const red_flash_config_t *config);
    void term();

public:
    WL_Flash *flash;

private:
    //
    // VFS interface
    //
    int get_free_fd();
    char *make_red_path(const char *path);

    static int map_red_errno(int err);

    static off_t lseek_p(void *ctx, int fd, off_t size, int mode);
    static ssize_t read_p(void *ctx, int fd, void *dst, size_t size);
    static int open_p(void *ctx, const char *path, int flags, int mode);
    static int close_p(void *ctx, int fd);
    static int fstat_p(void *ctx, int fd, struct stat *st);
    static int stat_p(void *ctx, const char *path, struct stat *st);

#if REDCONF_API_POSIX_READDIR == 1
    static DIR *opendir_p(void *ctx, const char *name);
    static struct dirent *readdir_p(void *ctx, DIR *pdir);
    static int readdir_r_p(void *ctx, DIR *pdir, struct dirent *entry, struct dirent **out_dirent);
    static long telldir_p(void *ctx, DIR *pdir);
    static void seekdir_p(void *ctx, DIR *pdir, long offset);
    static int closedir_p(void *ctx, DIR *pdir);
#endif // REDCONF_API_POSIX_READDIR == 1

#if REDCONF_READ_ONLY == 0
    static ssize_t write_p(void *ctx, int fd, const void *data, size_t size);
    static int fsync_p(void *ctx, int fd);

#if REDCONF_API_POSIX_LINK == 1
    static int link_p(void *ctx, const char *n1, const char *n2);
#endif // REDCONF_API_POSIX_LINK == 1

#if REDCONF_API_POSIX_UNLINK == 1
    static int unlink_p(void *ctx, const char *path);
#endif // REDCONF_API_POSIX_UNLINK == 1

#if REDCONF_API_POSIX_RENAME == 1
    static int rename_p(void *ctx, const char *src, const char *dst);
#endif // REDCONF_API_POSIX_RENAME == 1

#if REDCONF_API_POSIX_MKDIR == 1
    static int mkdir_p(void *ctx, const char *name, mode_t mode);
#endif // REDCONF_API_POSIX_MKDIR == 1

#if REDCONF_API_POSIX_RMDIR == 1
    static int rmdir_p(void *ctx, const char *name);
#endif // REDCONF_API_POSIX_RMDIR == 1
#endif // REDCONF_READ_ONLY == 0

    //
    // Flash_Access implementation
    //
    virtual size_t chip_size();
    virtual esp_err_t erase_sector(size_t sector);
    virtual esp_err_t erase_range(size_t start_address, size_t size);
    virtual esp_err_t write(size_t dest_addr, const void *src, size_t size);
    virtual esp_err_t read(size_t src_addr, void *dest, size_t size);
    virtual size_t sector_size();

private:
    red_flash_config_t cfg;
    const esp_partition_t *part;

    int label_len;

    bool mounted;
    bool registered;

    size_t chip_sz;
    size_t sector_sz;
    size_t block_cnt;

    typedef struct vfs_fd
    {
        int32_t red_fd;
    } vfs_fd_t;

    vfs_fd_t *fds;
};

#endif
