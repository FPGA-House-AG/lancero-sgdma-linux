/* performs alternating write and reads to an address on the
 * control bus of Lancero PCIe IP
 *
 * Compile with: cc -std=c99 -o stress stress.c
 *
 * Leon Woestenberg <leon@sidebranch.com>
 */


#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
  
#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
  __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)
 
#define MAP_SIZE (32 * 1024UL)
#define MAP_MASK (MAP_SIZE - 1)

int main(int argc, char **argv) {
    int fd;
    void *map_base, *virt_addr; 
	unsigned long read_result, writeval;
	off_t target;
	int access_type = 'w';
	
	if(argc < 3) {
		fprintf(stderr, "\nUsage:\t%s device address ]\n"
			"\tdevice \tdevice name, for example: /dev/lancero\n",
			"\taddress\tmemory address to act upon\n",
			argv[0]);
		exit(1);
	}
        /* target address */
	target = strtoul(argv[2], 0, 0);

	if(argc > 2)
		access_type = tolower(argv[2][0]);

    if((fd = open(argv[1], O_RDWR | O_SYNC)) == -1) FATAL;
    printf("device opened.\n"); 
    fflush(stdout);
    
    /* Map one page */
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if(map_base == (void *) -1) FATAL;
    printf("Memory mapped at address %p.\n", map_base); 
    fflush(stdout);
    
    virt_addr = map_base + target;

#if 0
    read_result = *((unsigned long *) virt_addr);
    printf("Value at address 0x%08x (%p): 0x%08x\n", target, virt_addr, read_result); 
#endif

    int i = 0;
    while(1) {

      /* write value 0xbabecafe to control bus address 0 */
      virt_addr = map_base + 0;
      *((unsigned long *)virt_addr) = 0xbabecafe;

      /* read 'i' from control bus address 0 */
      i = *((unsigned long *)virt_addr);
    }
    if(munmap(map_base, MAP_SIZE) == -1) FATAL;
    close(fd);
    return 0;
}

