#define _BSD_SOURCE
#define _XOPEN_SOURCE 600
#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
//#include <malloc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

uint32_t lancero_read_register(int fd, off_t off)
{
  uint32_t value;
  int rc;
  rc = pread(fd, &value, 4, off);
  assert(rc == 4);
  return value;
}

void lancero_write_register(int fd, off_t off, uint32_t value)
{
  int rc;
  rc = pwrite(fd, &value, 4, off);
  assert(rc == 4);
}

/* reads and prints the status of the Avalon ST/MM adaptor
 * returns status register value
 */
uint32_t adaptor_status(int fd, off_t off)
{
  /* status, write_index and available are continuously updated by the FPGA,
   */
  uint32_t status =      lancero_read_register(fd, off + 0x04);
  uint32_t write_index = lancero_read_register(fd, off + 0x10);
  uint32_t read_index =  lancero_read_register(fd, off + 0x14);
  uint32_t available =   lancero_read_register(fd, off + 0x18);
  uint32_t threshold =   lancero_read_register(fd, off + 0x1c);
  printf("write_index = 0x%08x, read_index = 0x%08x, available = 0x%08x, threshold = 0x%08x\n", write_index ,read_index, available, threshold);
  printf("status = 0x%08x %s%s%s\n", status, status&4?"THRES ":"",  status&2?"EOP ":"",  status&1?"RUN ":"");
  return status;
}

int main(int argc, char* argv[])
{
  int rc;
  int size = 1024 * 1024;
  int threshold = size / 8;
  uint32_t value, value_stop, last, write_index, read_index = 0;

  char *filename = "out.bin";

  char *buffer = NULL;
  posix_memalign((void **)&buffer, 4096, size);
  assert(buffer);

  int file_fd = -1;
  int lancero_user_fd = open("lancero_user", O_RDWR);
  assert(lancero_user_fd >= 0);
  int lancero_control_fd = open("lancero_control", O_RDWR);
  assert(lancero_control_fd >= 0);
  int lancero_events_fd = open("lancero_events", O_RDWR);
  assert(lancero_events_fd >= 0);
  int lancero_sgdma_fd = open("lancero_sgdma", O_RDWR | O_NONBLOCK);
  assert(lancero_sgdma_fd >= 0);

#if 0
  lancero_write_register(lancero_control_fd, 0x208, 0x80000000);
#endif

  /* create file to write data to */
  if (filename) {
    file_fd = open(filename, O_RDWR | O_CREAT | O_TRUNC | O_SYNC, 0666);
    assert(file_fd >= 0);
  }

  value = lancero_read_register(lancero_control_fd, 0x200);
  if ((value & 0xffffff00) != 0x00c10000)
  fprintf(stdout, "Could not find Lancero write engine identifier.\n");

	/* reset Avalon ST packet generator */
	lancero_write_register(lancero_user_fd, 0x24, 0x0/*clear RUN*/);
	/* reset Avalon ST/MM adaptor */
	lancero_write_register(lancero_user_fd, 0x08, 0);
	/* flush previous write, to prevent out-of-order */
	value = lancero_read_register(lancero_user_fd, 0x08);

  /* configure the Avalon ST/MM adaptor */
  lancero_write_register(lancero_user_fd, 0x0c, size);
  lancero_write_register(lancero_user_fd, 0x1c, threshold);
  /* start Avalon ST/MM adaptor */
  lancero_write_register(lancero_user_fd, 0x08, 4/*IE_THRESHOLD_FILLED*/ | 2/*IE_EOP_SEEN */ | 1/*RUN*/);
  /* flush previous write, to prevent out-of-order */
  (void)lancero_read_register(lancero_user_fd, 0x08);
  lancero_write_register(lancero_user_fd, 0x14, 0/* read index*/);

  rc = read(lancero_sgdma_fd, buffer, size);
  printf("read() = %d\n", rc);
  assert(rc == size);

  value = 0;
  last = -1;
  /* poll for write engine to have been started */
  printf("Polling write engine control, waiting for RUN...\n");
  while ((value & 1) == 0)
  {
    value = lancero_read_register(lancero_control_fd, 0x208);
    if (value != last) printf("Write engine is %sRUNNING. (control = 0x%08x)\n", value & 1?"":"NOT ", value);
    last = value;
  }
  /* stop the engine (later) by clearing RUN (bit 0), leave other bits as-is */
  value_stop = value & (~0x1);
  //printf("Will stop write engine by writing 0x%08lx to control later.\n", value_stop);

  value = 0;
  last = -1;
  printf("Polling write engine status, waiting for BUSY...\n");
  /* poll for write engine to have become busy */
  while ((value & 1) == 0)
  {
    value = lancero_read_register(lancero_control_fd, 0x204);
    if (value != last) printf("Write engine is %sBUSY. (status = 0x%08x)\n", value & 1?"":"NOT ", value);
    last = value;
  }

  adaptor_status(lancero_user_fd, 0);
  adaptor_status(lancero_user_fd, 0);

  printf("Starting Avalon ST packet generator.\n");
  /* configure the Avalon ST packet generator */
  lancero_write_register(lancero_user_fd, 0x2c, 1/*packet length*/);
  lancero_write_register(lancero_user_fd, 0x30, 0xffff/*number of packets within SOP/EOP*/);
  lancero_write_register(lancero_user_fd, 0x34, 20/*idle between packets*/);
  /* flush previous write, to prevent out-of-order */
  (void)lancero_read_register(lancero_user_fd, 0x24);
  /* start Avalon ST packet generator */
  lancero_write_register(lancero_user_fd, 0x24, 0x1/*set RUN*/);
  /* flush previous write, to prevent out-of-order */
  (void)lancero_read_register(lancero_user_fd, 0x24);

  adaptor_status(lancero_user_fd, 0);
  adaptor_status(lancero_user_fd, 0);

  while (1) {
    uint32_t available = lancero_read_register(lancero_user_fd, 0x18);
    printf("available = 0x%08x = %d bytes\n", available, available);
		while (available >= threshold) {
			printf("Processing %d bytes.\n", available);
			/* { here is where the application can process the bytes
			 *   between readIndex and readIndex + available }
			 */
			/* increase read index */
			read_index = (read_index + available) % size;
			/* publish read index to the adapter, so that it updates
			 * its available register and interrupt status */
			lancero_write_register(lancero_user_fd, 0x14, read_index);
			/* see how much bytes are available beyond the readIndex */
			available = lancero_read_register(lancero_user_fd, 0x18);
			printf("available = 0x%08x = %d bytes\n", available, available);
			/* end of data stream */
		}
    uint32_t status = adaptor_status(lancero_user_fd, 0);
		if (status & 2/*EOP*/) {
			uint32_t available = lancero_read_register(lancero_user_fd, 0x18);
			/* see how much bytes are available beyond the readIndex */
			printf("EOP; Processing last %d bytes.\n", available);
			/* reset Avalon ST packet generator */
			lancero_write_register(lancero_user_fd, 0x24, 0x0/*clear RUN*/);
			/* reset Avalon ST packet generator */
			lancero_write_register(lancero_user_fd, 0x24, 0x1/*RUN*/);
			break;
		}
		printf("Waiting for threshold event.\n");
		/* block until an interrupt event occurs */
		read(lancero_events_fd, &value, 4);
		printf("events = 0x%08lx\n", value);
	}

  printf("Stopping write engine, setting control register to 0x%08x.\n", value_stop);
  /* stop engine */
  lancero_write_register(lancero_control_fd, 0x208, value_stop);
  value = lancero_read_register(lancero_control_fd, 0x208);
  printf("Control register is 0x%08x.\n", value);

  value = 1;
  printf("Polling write engine status, waiting for IDLE...\n");
  /* poll for write engine to have become busy */
  while ((value & 1) == 1)
  {
    value = lancero_read_register(lancero_control_fd, 0x204);
    if (value != last) printf("Write engine status is %sBUSY. (status = 0x%08x)\n", value & 1?"":"NOT ", value);
    last = value;
  }
  printf("Status register is 0x%08x.\n", value);
  if (value == 0x00000040)
    printf("SUCCESFULL\n");

#if 0
  /* reset engine */
  lancero_write_register(lancero_control_fd, 0x208, 0x80000000);
#endif

  /* reset Avalon ST packet generator */
  lancero_write_register(lancero_user_fd, 0x24, 0x0/*clear RUN*/);
  /* reset Avalon ST/MM adaptor */
  lancero_write_register(lancero_user_fd, 0x08, 0);
  /* flush previous write, to prevent out-of-order */
  value = lancero_read_register(lancero_control_fd, 0x204);

#if 0
  while (count--) {
    /* select Avalon MM address */
    off_t off = lseek(fpga_fd, addr, SEEK_SET);
    /* read data from Avalon MM into buffer using SGDMA */
    rc = read(fpga_fd, buffer, size);
    assert(rc == size);
    /* file argument given? */
    if (file_fd >= 0) {
      /* write buffer to file */
      rc = write(file_fd, buffer, size);
      assert(rc == size);
    }
  }
#endif
  close(lancero_sgdma_fd);
  close(lancero_events_fd);
  close(lancero_control_fd);
  close(lancero_sgdma_fd);
  if (file_fd >=0) {
    close(file_fd);
  }
  
  free(buffer);
}
