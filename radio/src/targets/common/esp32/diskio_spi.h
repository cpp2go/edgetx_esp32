#ifndef _SDIO_SPI_H_
#define _SDIO_SPI_H_

#include <hal/fatfs_diskio.h>
#include "sdmmc_cmd.h"

extern const diskio_driver_t sdcard_spi_driver;
sdmmc_card_t* sdcard_spi_get_card(void);

#endif // _SDIO_SPI_H_