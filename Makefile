#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := lulupet_poc

SOLUTION_PATH ?= $(abspath $(shell pwd))/../espidf4.0-esp-who

include $(SOLUTION_PATH)/components/component_conf.mk
include $(IDF_PATH)/make/project.mk

