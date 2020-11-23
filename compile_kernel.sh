set -e

LINUX_PATH=/home/johndoe/linux
CONFIGS_PATH=/home/johndoe/rpi_config

cp -v $CONFIGS_PATH/.config $LINUX_PATH/.config

make -j4 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage modules dtbs

