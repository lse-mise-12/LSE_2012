rmmod spi
insmod /home/jvillena/spi.o
mknod /dev/spi_if c 70 0
chmod 666 /dev/spi_if
