set -e

CONFIGS_PATH=/home/johndoe/rpi_config
ROOTFS_PATH=/media/johndoe/rootfs2

cp -v $CONFIGS_PATH/asound.conf $ROOTFS_PATH/etc/asound.conf
cp -v $CONFIGS_PATH/asound.state $ROOTFS_PATH/var/lib/alsa/asound.state

mkdir -v -p $ROOTFS_PATH/etc/gpio
mkdir -v -p $ROOTFS_PATH/etc/ppp/chat
mkdir -v -p $ROOTFS_PATH/etc/ppp/peers
mkdir -v -p $ROOTFS_PATH/etc/gsm

cp -v $CONFIGS_PATH/gpio.sh /media/johndoe/rootfs2/etc/gpio/gpio.sh
cp -v $CONFIGS_PATH/rc.local /media/johndoe/rootfs2/etc/rc.local
cp -v $CONFIGS_PATH/a-gsm_chat /media/johndoe/rootfs2/etc/ppp/chat/a-gsm
cp -v $CONFIGS_PATH/a-gsm_peers /media/johndoe/rootfs2/etc/ppp/peers/a-gsm
cp -v $CONFIGS_PATH/gsm_modem.sh /media/johndoe/rootfs2/etc/gsm/gsm_modem.sh

chmod -v +x $ROOTFS_PATH/etc/gpio/gpio.sh
chmod -v +x $ROOTFS_PATH/etc/gsm/gsm_modem.sh

