# Reliance Edge filesystem for the ESP32
RedFlash is a port of [Reliance Edge filesystem](https://github.com/datalightinc/reliance-edge) to the ESP32,
providing support for usage on internal and external flash.  External support
utilizes my [ExtFlash](https://github.com/lllucius/esp32_extflash) component.

To clone this repo, use the following to automatically include the extflash
and Reliance Edge submodules:
```
git clone --recurse-submodules https://github.com/datalightinc/reliance-edge.git
```

To use, just add the "extflash" and "redflash" components to your
components directory and initialize it with something like:

```
#include "redflash.h"

void app_main()
{
    esp_err_t err;

#if !defined(CONFIG_REDFS_PARTITION_LABEL)
    ext_flash_config_t ext_cfg =
    {
        .vspi = true,
        .sck_io_num = PIN_SPI_SCK,
        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = PIN_SPI_MOSI,
        .ss_io_num = PIN_SPI_SS,
        .hd_io_num = PIN_SPI_HD,
        .wp_io_num = PIN_SPI_WP,
        .speed_mhz = 40,
        .dma_channel = 1,
        .queue_size = 4,
        .max_dma_size = 8192,
        .sector_size = 0,
        .capacity = 0,
    };

    err = extflash.init(&ext_cfg);
    if (err != ESP_OK)
    {
        ...
    }
#endif

    const red_flash_config_t red_cfg =
    {
#if !defined(CONFIG_REDFS_PARTITION_LABEL)
        .flash = &extflash,
        .part_label = NULL,
#else
        .flash = NULL,
        .part_label = CONFIG_REDFS_PARTITION_LABEL,
#endif
        .volume_label = VOLUME_LABEL,
        .base_path = MOUNT_POINT,
        .open_files = openfiles,
        .auto_format = true,
    };

    err = redflash.init(&red_cfg);
    if (err != ESP_OK)
    {
        ...
    }
    ...
}
```

The configuration options for RedFlash are:

```
typedef struct
{
    ExtFlash *flash;            // initialized ExtFlash, or NULL for internal flash
    const char *part_label;     // partition label if using internal flash
    const char *volume_label;   // RedFS volume label from gaRedVolConf
    const char *base_path;      // mount point
    int open_files;             // number of open files to support
    bool auto_format;           // true=format if not valid
} red_flash_config_t;
```

You will also need to use the Reliance Edge Configuration Tool found
here (Datalight Extras)[https://www.datalight.com/resources/extras] to
create the "redconf.c" and "redconf.h" files found in the root of this
repo.

More documentation to follow.

