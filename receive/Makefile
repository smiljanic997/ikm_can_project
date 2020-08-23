#
TARGET = receive_can_example

#
CROSS_COMPILE = arm-linux-gnueabihf-

CFLAGS = -I${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include -I${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include/soc_a10 -I${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include/soc_a10/socal -Dsoc_a10
LDFLAGS = -g -Wall -lpthread
CC = $(CROSS_COMPILE)gcc
ARCH = arm


build: $(TARGET)
$(TARGET): receive_can_example.o
	$(CC) $(LDFLAGS)   $^ -o $@  
%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~ 
