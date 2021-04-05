#
# Copyright (C) 2014, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
# Kinetis Port Author David Sidrane <david_s5@nscdg.com>
#

LIBUAVCAN_KINETIS_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

LIBUAVCAN_KINETIS_SRC := $(shell find $(LIBUAVCAN_KINETIS_DIR)src -type f -name '*.cpp')

LIBUAVCAN_KINETIS_INC := $(LIBUAVCAN_KINETIS_DIR)include/
