#! /bin/sh
# /etc/init.d/installPulseDriver.sh
#
#sudo cp /home/pi/rfrpi_src/gpio-freq/installPulseDriver.sh /etc/init.d/installPulseDriver.sh
#sudo chmod 755 /etc/init.d/installPulseDriver.sh
#sudo chown root:root /etc/init.d/installPulseDriver.sh
#
#Then you need to register it to run at startup.
#sudo update-rc.d installPulseDriver.sh defaults

# Some things that run always
#touch /var/lock/blah

# Carry out specific functions when asked to by the system
case "$1" in
  start)
    echo "Starting script installPulseDriver "
		sudo insmod /home/pi/rfrpi_src/gpio-freq/gpio-freq.ko gpios=16,22,23,17,24
		sudo chown pi /dev/gpiofreq*
    
    ;;
  stop)
    echo "Stopping script installPulseDriver"
		sudo rmmod gpio_freq.ko
    ;;
  *)
    echo "Usage: /etc/init.d/installPulseDriver {start|stop}"
    exit 1
    ;;
esac

exit 0
