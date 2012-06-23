CROSS_COMPILE = arm-linux-
CC_PATH = /usr/local/opt/crosstool/arm-linux/gcc-3.3.4-glibc-2.3.2/bin

CPPFLAGS_D = -D__KERNEL__ -DMODULE -I/usr/local/opt/linux24/include/

CFLAGS_D = -Wall -Wstrict-prototypes -Wno-trigraphs -O2 \
         -fno-strict-aliasing -fno-common -fomit-frame-pointer \
         -mcpu=arm920t 

# quitada optimizacion -O2 (para debug)
CFLAGS = -Wall -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -fomit-frame-pointer -mcpu=arm920t -Wno-format-y2k -g -pthread -D_DEBUG_

all: main spi
main:
	$(CC_PATH)/$(CROSS_COMPILE)gcc $(CFLAGS) -o car ./car2.c ./mcp.c
spi:
	$(CC_PATH)/$(CROSS_COMPILE)gcc $(CPPFLAGS_D) $(CFLAGS_D) -c ./spi.c
	
clean:
	rm -f *.o 
	rm car
