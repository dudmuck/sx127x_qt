sx127x_qt
=========

QT GUI for evaluating [SX1272](http://semtech.com/apps/product.php?pn=sx1272&x=0&y=0)/[SX1276](http://semtech.com/apps/product.php?pn=sx1276&x=0&y=0) on Raspberry Pi

## Preparing Raspberry Pi for build

ensure your RPi is up to date http://elinux.org/index.php?title=Add_software_to_Raspberry_Pi

### install qt-creator

Follow [these instructions](https://wiki.qt.io/Apt-get_Qt4_on_the_Raspberry_Pi) for installing QT4 and qtcreator.

### first-time setup of qt-creator on RPi, and toolchain assignment

Also described in the [same instructions](https://wiki.qt.io/Apt-get_Qt4_on_the_Raspberry_Pi).

# wiringPi installation

install as described http://wiringpi.com/download-and-install/

# enable SPI on RPi
first time load spi driver as described https://projects.drogon.net/raspberry-pi/wiringpi/spi-library/

So it auto-loads on boot: edit /etc/modprobe.d/raspi-blacklist.conf

# build

Open sx127x.pro in qt-creator

Edit -> Build All
