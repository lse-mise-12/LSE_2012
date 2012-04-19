rmmod spi_cs
insmod /home/jvillena/spi_cs.o
#mknod /dev/spi_if c 70 0
#chmod 666 /dev/spi_if
mknod /root/spi_if c 70 0
chmod 666 /root/spi_if
