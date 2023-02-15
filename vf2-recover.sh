#!/bin/sh
#
# This script is used to recover the StarFive VisionFive 2 board
#
# Set the boot source DIP switches to UART.
# Start this program.
# Thereafter switch the board on.
#
# vf2-recover.sh [serial_device [path_to_recovery_files]
#
# serial_device
#     serial device defaults to /dev/ttyUSB0
#
# path_to_recovery_files
#     path to the prebuilt recovery binaries
#

PROG_NAME=./vf2-recover
UART_PORT=/dev/${1:-ttyUSB0}
RECOVERY_PATH=${2:-.}

RECOVERY_PATH=${RECOVERY_PATH}/jh7110-recovery-*.bin
SPL_PATH=${RECOVERY_PATH}/u-boot-spl.bin.normal.out
UBOOT_PATH=${RECOVERY_PATH}/visionfive2_fw_payload.img

${PROG_NAME} \
	-D ${UART_PORT} \
	-r ${RECOVERY_PATH} \
	-s ${SPL_PATH} \
	-u ${UBOOT_PATH}
