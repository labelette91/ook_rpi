sudo dmesg -c
sudo dmesg -c
make
sudo rmmod gpio-freq.ko
sudo insmod gpio-freq.ko gpios=16,22,23,17
sudo chown pi /dev/gpiofreq*
ls  /dev/gpiofreq*
cd test
make
cd ..
# ./test/test
dmesg 


