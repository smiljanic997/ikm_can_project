#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"

#include "header.h"
#include "offsets.h"

#define MEM_PATH "/dev/mem"

#define ALT_LWFPGASLVS_OFST 0xFF200000

#define HW_REGS_BASE ( 0xFC000000 )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))        // '!!' to make sure this returns 0 or 1
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))

uint32_t read_double_word(void* base_address, uint32_t offset) {
    return *((uint32_t*)(base_address + offset));
}

uint16_t read_first_word(void* base_address, uint32_t offset) {
    return (uint16_t)(read_double_word(base_address, offset));
}

uint16_t read_second_word(void* base_address, uint32_t offset) {
    return (uint16_t)(read_double_word(base_address, offset) >> 16);
}

void send_frame(void* txt_buffer_address, void* tx_command_address, void* data, uint8_t len, uint8_t is_fd) {
    uint32_t frame_format_word = 0;
    frame_format_word |= 4;
    frame_format_word |= (0 << 7);
    frame_format_word |= (1 << 9); // switch bit-rate; just for CAN fd
    *((uint32_t*)(txt_buffer_address)) = frame_format_word;

    uint32_t id_word = (0x100 << 18); // identifier: 0
    *((uint32_t*)(txt_buffer_address + 0x4)) = id_word;

    *((uint32_t*)(txt_buffer_address + 0x8)) = 0;
    *((uint32_t*)(txt_buffer_address + 0xC)) = 0;   // transmit asap

    *((uint32_t*)(txt_buffer_address + 0x10)) = *((uint32_t*)(data)); //set data

    uint32_t command = 0;
    command |= 0x2; // set ready command
    command |= (1 << 8);    // choose first txt buffer (TODO)
    *((uint32_t*)(tx_command_address)) = command;
}

void enable(void* base_address) {
    uint32_t settings_mode;
    settings_mode = read_double_word(base_address, 0x4);
    settings_mode |= (1 << 22);
    // settings_mode |= (1 << 21);
    *((uint32_t*)(base_address + 0x4)) = settings_mode;
}

void set_bit_rate(void* base_address) {
    uint32_t btr = 0;
    btr = (5 << 19); // time quanta = 5
    btr |= 7;       // prop = 29
    btr |= (8 << 7); // phase 1 = 20
    btr |= (4 << 13); // phase 2 = 20
    btr |= (3 << 27);  // sjw = 3
    *((uint32_t*)(base_address + 0x24)) = btr;

    uint32_t btr_fd = 0;
    btr_fd = (1 << 19); // time quanta = 1
    btr_fd |= 7;       // prop = 29
    btr_fd |= (8 << 7); // phase 1 = 20
    btr_fd |= (4 << 13); // phase 2 = 20
    btr_fd |= (3 << 27);  // sjw = 3
    *((uint32_t*)(base_address + 0x28)) = btr_fd;
}

void self_test_mode(void* base_address) {
    uint32_t temp;
    temp = read_first_word(base_address, 0x4);
    BIT_SET(temp, 2);
    BIT_CLEAR(temp, 4); // turn off fd support
    *((uint32_t*)(base_address + 0x4)) = temp;
}

void receive_frame(void* base_address) {
    while(1) {
        uint32_t rx_status = read_first_word(base_address, RX_STATUS_OFFSET);
        while(rx_status & 0x0 == 0) {
            // printf("pooling...\n");
            rx_status = read_first_word(base_address, RX_STATUS_OFFSET);
        }
        uint8_t data[64];
        uint32_t temp;
        uint32_t ffw = read_double_word(base_address, RX_DATA_OFFSET);
        uint32_t id = read_double_word(base_address, RX_DATA_OFFSET);
        id = id >> 18; // 18 is for base id (TODO: this should be checked first)
        uint32_t ts_low = read_double_word(base_address, RX_DATA_OFFSET);
        uint32_t ts_high = read_double_word(base_address, RX_DATA_OFFSET);
        uint32_t rwcnt = (ffw >> 11) & 0x1F;
        uint8_t data_len = rwcnt - 3; // len without timestamp and id fields
        if(data_len > 0) {
            int i;
            uint8_t arr[8];
            for(i = 0; i < data_len; i++) {
                temp = read_double_word(base_address, RX_DATA_OFFSET);
                arr[i * 4] = temp & 0xFF;
                arr[i * 4 + 1] = (temp >> 8) & 0xFF;
                arr[i * 4 + 2] = (temp >> 16) & 0xFF;
                arr[i * 4 + 3] = (temp >> 24) & 0xFF;
            }
            printf("ID: 0x%.3lX  ", id);
            for(i = 0; i < 8; i++) {
                printf("0x%.2lX ", arr[i]);
                fflush(stdout);
            }
            printf("\n");
        }
    }
}

int main() {
    void* lw_virtual_base;
    void* leds_virtual_addr;
    void* can_virtual_addr;
    int fd_mem;

    if ((fd_mem = open(MEM_PATH, (O_RDWR | O_SYNC))) == -1) {
		printf("Can't open %s!\n", MEM_PATH);
		
		return 1;
	}

    lw_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd_mem, HW_REGS_BASE);

    if (lw_virtual_base == MAP_FAILED) {
		printf("mmap() failed\n");
		return 1;
	}

    leds_virtual_addr = lw_virtual_base +
	    	((unsigned long)(ALT_LWFPGASLVS_OFST + PIO_0_BASE) &
	    	(unsigned long)(HW_REGS_MASK));

    can_virtual_addr = lw_virtual_base +
                ((unsigned long)(ALT_LWFPGASLVS_OFST + CTU_CANFD_AVALON_0_BASE) &
	    	    (unsigned long)(HW_REGS_MASK));
    
    set_bit_rate(can_virtual_addr);
    enable(can_virtual_addr);

    receive_frame(can_virtual_addr);
}
