APPLICATION = test_senseair_sunrise
BOARD ?= lora3a-h10
RIOTBASE ?= $(CURDIR)/../RIOT
LORA3ABASE ?= $(CURDIR)/../lora3a-boards
EXTERNAL_BOARD_DIRS=$(LORA3ABASE)/boards
EXTERNAL_MODULE_DIRS=$(LORA3ABASE)/modules
EXTERNAL_PKG_DIRS=$(LORA3ABASE)/pkg
QUIET ?= 1
DEVELHELP ?= 1
PORT ?= /dev/ttyUSB0

USEMODULE += printf_float
USEMODULE += senseair
USEMODULE += fram
USEMODULE += saml21_cpu_debug
USEMODULE += saml21_backup_mode
USEMODULE += periph_i2c_reconfigure
USEMODULE += ztimer_msec
USEMODULE += saul_default
USEMODULE += od_string

CFLAGS += -DENABLE_ACME1=MODE_I2C -DSENSEAIR_I2C_DEV=I2C_DEV\(1\) -DSENSEAIR_ENABLE_PIN=GPIO_PIN\(PB,23\)

include $(RIOTBASE)/Makefile.include
