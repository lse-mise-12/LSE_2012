CROSS_COMPILE = arm-linux-
CC_PATH = /usr/local/opt/crosstool/arm-linux/gcc-3.3.4-glibc-2.3.2/bin

CPPFLAGS_D = -D__KERNEL__ -DMODULE -I/usr/local/opt/linux24/include/

CFLAGS_D = -Wall -Wstrict-prototypes -Wno-trigraphs -O2 \
         -fno-strict-aliasing -fno-common -fomit-frame-pointer \
         -mcpu=arm920t

CFLAGS = -Wall -Wstrict-prototypes -Wno-trigraphs -O2 -fno-strict-aliasing -fno-common -fomit-frame-pointer -mcpu=arm920t

all: main spi
main:
	$(CC_PATH)/$(CROSS_COMPILE)gcc $(CFLAGS) -o main main.c mcp.c
spi:
	$(CC_PATH)/$(CROSS_COMPILE)gcc $(CPPFLAGS_D) $(CFLAGS_D) -c spi_cs.c
	
clean:
	rm -f *.o *~ core
	rm main
