/*
 * touch_sub_spi.h
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef TOUCH_SPI_H
#define TOUCH_SPI_H

#include <touch_sub_hwif.h>

extern int touch_sub_spi_read(struct spi_device *spi, struct touch_sub_bus_msg *msg);
extern int touch_sub_spi_write(struct spi_device *spi, struct touch_sub_bus_msg *msg);
extern int touch_sub_spi_xfer(struct spi_device *spi, struct touch_sub_xfer_msg *xfer);
extern int touch_sub_spi_device_init(struct touch_sub_hwif *hwif, void *driver);
extern void touch_sub_spi_device_exit(struct touch_sub_hwif *hwif);

#if defined(CONFIG_SUB_SECURE_TOUCH) && (0)
extern void touch_sub_spi_set(struct touch_sub_core_data *ts);
extern int touch_sub_spi_get(struct touch_sub_core_data *ts);
#endif

#endif
