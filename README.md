sx127x_qt
=========

QT GUI for evaluating [SX1272](http://semtech.com/apps/product.php?pn=sx1272&x=0&y=0)/[SX1276](http://semtech.com/apps/product.php?pn=sx1276&x=0&y=0) on Raspberry Pi

## Preparing Raspberry Pi for build

ensure your RPi is up to date http://elinux.org/Add_software

### install qt-creator

sudo apt-get install qtcreator

### first-time setup of qt-creator on RPi

- Go to menu "help" -> "plugins"
- Uncheck "device support" -> "remote linux"
- Restart Qt Creator
- Go to "tools" "options" tab "Build & Run" -> "Qt versions" -> add "/usr/bin/qmake-qt4" again.

http://qt-project.org/wiki/apt-get_Qt4_on_the_Raspberry_Pi

# wiringPi installation

install as described http://wiringpi.com/download-and-install/

# enable SPI on RPi
first time load spi driver as described https://projects.drogon.net/raspberry-pi/wiringpi/spi-library/

So it auto-loads on boot: edit /etc/modprobe.d/raspi-blacklist.conf

# build

Open sx127x.pro in qt-creator

Edit -> Build All
