/* gcc -std=c99 -D_GNU_SOURCE -o sgdma sgdma.c */

#include <assert.h>
#include <stdio.h>
#include <stdint.h>
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

#define DEVNODE "/248"

#define BUF_SIZE (1024)
  
int main(int argc, char **argv) {

  /* open the device file node that corresponds with the SGDMA buses */
  int fd;
  fd = open(DEVNODE, O_RDWR);
  if (fd < 0) {
    printf("Could not open " DEVNODE "\n");
    exit(-1);
  }
  printf(DEVNODE " opened.\n"); 
    
  /* allocate a buffer. this buffer is always allocated in virtual
   * memory and may lie scattered across memory */

  unsigned char *buffer = malloc(1024);
  *(uint64_t *)buffer = 0x01234567890ABCDEF;

  /* write BUF_SIZE bytes to the FPGA, to a specific position in the device file.
   * This position corresponds one-to-one with the address (0x80000000 here).
   *
   * Note that, in reality, the read engine is asked to read from the processor
   * memory through direct memory access. This is transparent to the application.
   */
  int rc = pwrite(fd, buffer, BUF_SIZE, 0x80000000UL);
  if (rc != BUF_SIZE) {
    printf("Error writing to " DEVNODE "\n");
  }
  free(buffer);
  close(fd);
}

