set -e

KERNEL_PATH=/home/johndoe/linux/arch/arm/boot
CONFIGS_PATH=/home/johndoe/rpi_config
BOOT_PATH=/media/johndoe/boot

cp -v $KERNEL_PATH/zImage $BOOT_PATH/kernel7.img
cp -v $KERNEL_PATH/dts/*.dtb $BOOT_PATH/
cp -v $KERNEL_PATH/dts/overlays/*.dtb* $BOOT_PATH/overlays/

cp -v $CONFIGS_PATH/config.txt $BOOT_PATH/config.txt
cp -v $CONFIGS_PATH/dt-blob.bin $BOOT_PATH/dt-blob.bin
cp -v $CONFIGS_PATH/dt-blob-cam1.dts $BOOT_PATH/dt-blob-cam1.dts

