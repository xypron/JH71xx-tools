# JH71xx-tools
Bootloader recovery and updater tool for StarFive JH7100 SoCs.

![Screenshot](https://github.com/kprasadvnsi/JH71xx-tools/images/screenshot.png)

Command Line Parameters
-----------------------
    -D, --device <tty device>      : Serial tty device path.
    -r, --recovery <filename>      : Bootloader recovery firmware.
    -b, --bootloader <filename>    : Second stage bootloader.
    -d, --ddrinit <filename>       : DRAM initialization firmware.
    -h, --help                     : Show this help.

## Build
```
gcc -o jh7100-recover jh7100-recover.c
```

## Examples

1. Update second stage bootloader.

```
$ ./jh7100-recover -D /dev/ttyUSB0 \
-r vic_second_boot.bin \
-b bootloader-BEAGLEV-buildroot.bin.out
```
2. Update DRAM initialization firmware.

```
$ ./jh7100-recover -D /dev/ttyUSB0 \
-r vic_second_boot.bin \
-d ddrinit-2133-buildroot.bin.out
```

3. Update both second stage bootloader and DRAM initialization firmware.

```
$ ./jh7100-recover -D /dev/ttyUSB0 \
-r vic_second_boot.bin \
-b bootloader-BEAGLEV-buildroot.bin.out \
-d ddrinit-2133-buildroot.bin.out
```