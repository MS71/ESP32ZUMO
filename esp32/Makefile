#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

EXCLUDE_COMPONENTS := $IDF_PATH/../components
PROJECT_NAME := app-ota-template

include $(IDF_PATH)/make/project.mk

ota_zumoros: app
	curl 192.168.1.72:8032 --data-binary @- < build/$(PROJECT_NAME).bin
	
ota: app
