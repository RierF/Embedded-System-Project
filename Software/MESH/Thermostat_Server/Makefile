PROJECT_NAME     := PWM
TARGETS          := nrf52832_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../../nRF5_SDK_15.0.0_a53641a
MESH_SDK_ROOT := ../../../nrf5_sdk_for_mesh
PROJ_DIR := src

$(OUTPUT_DIRECTORY)/nrf52832_xxaa.out: \
  LINKER_SCRIPT  := gcc_nrf52.ld


# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52.S \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(PROJ_DIR)/main.c \
  $(MESH_SDK_ROOT)/examples/common/src/mesh_provisionee.c \
  $(MESH_SDK_ROOT)/examples/common/src/app_onoff.c \
  $(MESH_SDK_ROOT)/examples/common/src/app_level.c \
  $(MESH_SDK_ROOT)/examples/common/src/rtt_input.c \
  $(MESH_SDK_ROOT)/examples/common/src/mesh_app_utils.c \
  $(MESH_SDK_ROOT)/examples/common/src/mesh_adv.c \
  $(MESH_SDK_ROOT)/examples/nrf_mesh_weak.c \
  $(MESH_SDK_ROOT)/examples/common/src/app_error_weak.c \
  $(MESH_SDK_ROOT)/examples/common/src/assertion_handler_weak.c \
  $(MESH_SDK_ROOT)/examples/common/src/assertion_handler_weak.c \
  $(MESH_SDK_ROOT)/examples/common/src/assertion_handler_weak.c \
  $(MESH_SDK_ROOT)/mesh/core/src/internal_event.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_mesh_configure.c \
  $(MESH_SDK_ROOT)/mesh/core/src/aes.c \
  $(MESH_SDK_ROOT)/mesh/core/src/msg_cache.c \
  $(MESH_SDK_ROOT)/mesh/core/src/transport.c \
  $(MESH_SDK_ROOT)/mesh/core/src/event.c \
  $(MESH_SDK_ROOT)/mesh/core/src/packet_buffer.c \
  $(MESH_SDK_ROOT)/mesh/core/src/flash_manager_defrag.c \
  $(MESH_SDK_ROOT)/mesh/core/src/fifo.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_flash.c \
  $(MESH_SDK_ROOT)/mesh/core/src/packet_mgr.c \
  $(MESH_SDK_ROOT)/mesh/core/src/net_state.c \
  $(MESH_SDK_ROOT)/mesh/core/src/mesh_flash.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_mesh_utils.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_mesh.c \
  $(MESH_SDK_ROOT)/mesh/core/src/queue.c \
  $(MESH_SDK_ROOT)/mesh/core/src/hal.c \
  $(MESH_SDK_ROOT)/mesh/core/src/aes_cmac.c \
  $(MESH_SDK_ROOT)/mesh/core/src/timer_scheduler.c \
  $(MESH_SDK_ROOT)/mesh/core/src/timer.c \
  $(MESH_SDK_ROOT)/mesh/core/src/rand.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_mesh_opt.c \
  $(MESH_SDK_ROOT)/mesh/core/src/mesh_opt.c \
  $(MESH_SDK_ROOT)/mesh/core/src/timeslot.c \
  $(MESH_SDK_ROOT)/mesh/core/src/bearer_event.c \
  $(MESH_SDK_ROOT)/mesh/core/src/enc.c \
  $(MESH_SDK_ROOT)/mesh/core/src/network.c \
  $(MESH_SDK_ROOT)/mesh/core/src/net_packet.c \
  $(MESH_SDK_ROOT)/mesh/core/src/msqueue.c \
  $(MESH_SDK_ROOT)/mesh/core/src/nrf_mesh_keygen.c \
  $(MESH_SDK_ROOT)/mesh/core/src/cache.c \
  $(MESH_SDK_ROOT)/mesh/core/src/list.c \
  $(MESH_SDK_ROOT)/mesh/core/src/log.c \
  $(MESH_SDK_ROOT)/mesh/core/src/flash_manager.c \
  $(MESH_SDK_ROOT)/mesh/core/src/ccm_soft.c \
  $(MESH_SDK_ROOT)/mesh/core/src/toolchain.c \
  $(MESH_SDK_ROOT)/mesh/core/src/replay_cache.c \
  $(MESH_SDK_ROOT)/mesh/core/src/beacon.c \
  $(MESH_SDK_ROOT)/mesh/core/src/flash_manager_internal.c \
  $(MESH_SDK_ROOT)/mesh/core/src/core_tx.c \
  $(MESH_SDK_ROOT)/mesh/core/src/heartbeat.c \
  $(MESH_SDK_ROOT)/mesh/core/src/net_beacon.c \
  $(MESH_SDK_ROOT)/mesh/core/src/mesh_config.c \
  $(MESH_SDK_ROOT)/mesh/core/src/fsm.c \
  $(MESH_SDK_ROOT)/mesh/core/src/mesh_config_backend.c \
  $(MESH_SDK_ROOT)/mesh/core/src/mesh_config_flashman_glue.c \
  $(MESH_SDK_ROOT)/mesh/core/src/core_tx_adv.c \
  $(MESH_SDK_ROOT)/mesh/stack/src/mesh_stack.c \
  $(MESH_SDK_ROOT)/mesh/gatt/src/mesh_gatt.c \
  $(MESH_SDK_ROOT)/mesh/gatt/src/proxy.c \
  $(MESH_SDK_ROOT)/mesh/gatt/src/proxy_filter.c \
  $(MESH_SDK_ROOT)/mesh/access/src/access_publish.c \
  $(MESH_SDK_ROOT)/mesh/access/src/access.c \
  $(MESH_SDK_ROOT)/mesh/access/src/access_reliable.c \
  $(MESH_SDK_ROOT)/mesh/access/src/access_loopback.c \
  $(MESH_SDK_ROOT)/mesh/access/src/device_state_manager.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/ad_listener.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/ad_type_filter.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/adv_packet_filter.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/advertiser.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/bearer_handler.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/broadcast.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/filter_engine.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/gap_address_filter.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/radio_config.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/rssi_filter.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/scanner.c \
  $(MESH_SDK_ROOT)/mesh/bearer/src/mesh_pa_lna.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/nrf_mesh_prov_bearer_gatt.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/prov_provisionee.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/nrf_mesh_prov.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/provisioning.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/prov_beacon.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/prov_utils.c \
  $(MESH_SDK_ROOT)/mesh/prov/src/prov_bearer_adv.c \
  $(MESH_SDK_ROOT)/external/rtt/src/SEGGER_RTT.c \
  $(MESH_SDK_ROOT)/external/rtt/src/SEGGER_RTT_printf.c \
  $(MESH_SDK_ROOT)/external/micro-ecc/uECC.c \
  $(MESH_SDK_ROOT)/external/sdk_fix/nrf_sdh.c \
  $(MESH_SDK_ROOT)/models/foundation/config/src/composition_data.c \
  $(MESH_SDK_ROOT)/models/foundation/config/src/config_server.c \
  $(MESH_SDK_ROOT)/models/foundation/config/src/packed_index_list.c \
  $(MESH_SDK_ROOT)/models/foundation/health/src/health_server.c \
  $(MESH_SDK_ROOT)/models/model_spec/generic_onoff/src/generic_onoff_client.c \
  $(MESH_SDK_ROOT)/models/model_spec/generic_level/src/generic_level_client.c \
  $(MESH_SDK_ROOT)/models/model_spec/generic_onoff/src/generic_onoff_server.c \
  $(MESH_SDK_ROOT)/models/model_spec/generic_level/src/generic_level_server.c \
  $(MESH_SDK_ROOT)/models/model_spec/common/src/model_common.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/libraries/crc16/crc16.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_sd.c \
  $(SDK_ROOT)/components/libraries/experimental_memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/hardfault/nrf52/handler/hardfault_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/fifo/app_fifo.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/nfc/t4t_lib/hal_t4t/hal_nfc_t4t.c \
  $(SDK_ROOT)/components/nfc/ndef/generic/message/nfc_ndef_msg.c \
  $(SDK_ROOT)/components/nfc/ndef/generic/record/nfc_ndef_record.c \
  $(SDK_ROOT)/components/nfc/ndef/uri/nfc_uri_msg.c \
  $(SDK_ROOT)/components/nfc/ndef/uri/nfc_uri_rec.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c \
  $(SDK_ROOT)/modules/nrfx/hal/nrf_nvmc.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_power_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_SES.c \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd/nrf_nvic.c \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd/nrf_soc.c \
# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/libraries/experimental_memobj \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/softdevice/s132/headers \
  $(SDK_ROOT)/components/nfc/t4t_lib/hal_t4t \
  $(SDK_ROOT)/components/nfc/t4t_lib \
  config \
  include \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/nfc/ndef/uri \
  $(SDK_ROOT)/components/nfc/ndef/generic/message \
  $(SDK_ROOT)/components/nfc/ndef/generic/record \
  $(SDK_ROOT)/components/libraries/experimental_log \
  $(SDK_ROOT)/components/libraries/experimental_log/src \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
  $(SDK_ROOT)/components/softdevice/s132 \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/toolchain/cmsis/dsp/GCC \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(PROJ_DIR) \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(MESH_SDK_ROOT)/examples/common/include \
  $(MESH_SDK_ROOT)/external/rtt/include \
  $(MESH_SDK_ROOT)/external/micro-ecc \
  $(MESH_SDK_ROOT)/models/model_spec/generic_onoff/include \
  $(MESH_SDK_ROOT)/models/model_spec/generic_level/include \
  $(MESH_SDK_ROOT)/models/model_spec/common/include \
  $(MESH_SDK_ROOT)/models/foundation/config/include \
  $(MESH_SDK_ROOT)/models/foundation/health/include \
  $(MESH_SDK_ROOT)/mesh/core/api \
  $(MESH_SDK_ROOT)/mesh/core/include \
  $(MESH_SDK_ROOT)/mesh/access/include \
  $(MESH_SDK_ROOT)/mesh/access/api \
  $(MESH_SDK_ROOT)/mesh/dfu/api \
  $(MESH_SDK_ROOT)/mesh/dfu/include \
  $(MESH_SDK_ROOT)/mesh/prov/include \
  $(MESH_SDK_ROOT)/mesh/prov/api \
  $(MESH_SDK_ROOT)/mesh/bearer/api \
  $(MESH_SDK_ROOT)/mesh/bearer/include \
  $(MESH_SDK_ROOT)/mesh/gatt/include \
  $(MESH_SDK_ROOT)/mesh/gatt/api \
  $(MESH_SDK_ROOT)/mesh/stack/api \


# Libraries common to all targets
LIB_FILES += \
  $(SDK_ROOT)/components/nfc/t4t_lib/nfc_t4t_lib_gcc.a \
# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_PCA10040
CFLAGS += -DNO_VTOR_CONFIG
CFLAGS += -DUSE_APP_CONFIG
CFLAGS += -DMESH_FEATURE_GATT=1
CFLAGS += -DGATT_PROXY=1
CFLAGS += -DCONFIG_APP_IN_CORE
CFLAGS += -DNRF_LOG_USES_RTT=1
CFLAGS += -DNRF_MESH_LOG_ENABLE=NRF_LOG_USES_RTT
CFLAGS += -DNRF52_SERIES
CFLAGS += -DNRF52832
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52_PAN_74
CFLAGS += -DNRF_SD_BLE_API_VERSION=6
CFLAGS += -DS132
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs -std=gnu99
CFLAGS += -Wall -Werror -Wno-unused-variable -Wno-format -Wno-unused-value
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -DuECC_OPTIMIZATION_LEVEL=2
CFLAGS += -DuECC_SUPPORTS_secp160r1=0
CFLAGS += -DuECC_SUPPORTS_secp192r1=0
CFLAGS += -DuECC_SUPPORTS_secp224r1=0
CFLAGS += -DuECC_SUPPORTS_secp256r1=1
CFLAGS += -DuECC_SUPPORTS_secp256k1=0
CFLAGS += -DuECC_SUPPORT_COMPRESSED_POINT=0
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52_PAN_74
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=6
ASMFLAGS += -DS132
ASMFLAGS += -DSOFTDEVICE_PRESENT

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

nrf52832_xxaa: CFLAGS += -D__HEAP_SIZE=8192
nrf52832_xxaa: CFLAGS += -D__STACK_SIZE=8192
nrf52832_xxaa: ASMFLAGS += -D__HEAP_SIZE=8192
nrf52832_xxaa: ASMFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52832_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52832_xxaa
	@echo		flash_softdevice
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex
	@echo Flashing: $<
	nrfjprog -f nrf52 --program $< --sectorerase
	nrfjprog -f nrf52 --reset

# Flash softdevice
flash_softdevice:
	@echo Flashing: s132_nrf52_6.0.0_softdevice.hex
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_6.0.0_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset

flash_tag_connect:
	nrf52prog -s $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_6.0.0_softdevice.hex -x $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex

erase:
	nrfjprog -f nrf52 --eraseall
	
reset:
	nrfjprog -r

SDK_CONFIG_FILE := config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
	
rebuild:
	make VERBOSE=1 clean
	make VERBOSE=1 default
	
reflash:
	make VERBOSE=1 clean
	make VERBOSE=1 default
	make flash_softdevice
	make flash
