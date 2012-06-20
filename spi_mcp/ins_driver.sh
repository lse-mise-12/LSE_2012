rmmod spi
insmod /home/jvillena/spi.o
mknod /root/spi_if c 70 0
chmod 666 /root/spi_if
