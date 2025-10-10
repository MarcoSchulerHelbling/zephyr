# ST Microelectronics H3LIS331DL IMU sensor driver
#
# Copyright (c) 2025 Helbling Technik AG
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(h3lis331dl.c)
# zephyr_library_sources_ifdef(CONFIG_H3LIS331DL_SENSORHUB  h3lis331dl_shub.c)
# zephyr_library_sources_ifdef(CONFIG_H3LIS331DL_TRIGGER    h3lis331dl_trigger.c)

zephyr_library_include_directories(../stmemsc)
