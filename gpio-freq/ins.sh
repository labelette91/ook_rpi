sudo rmmod gpio-freq.ko
sudo insmod gpio-freq.ko gpios=16,22,23,17,24
sudo chown pi /dev/gpiofreq*
ls  /dev/gpiofreq*
