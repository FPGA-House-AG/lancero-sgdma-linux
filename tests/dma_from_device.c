#define _BSD_SOURCE
#define _XOPEN_SOURCE 500
#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

static int verbosity = 0;
static int read_back = 0;
static int allowed_accesses = 1;

static struct option const long_opts[] =
{
  {"device", required_argument, NULL, 'd'},
  {"address", required_argument, NULL, 'a'},
  {"size", required_argument, NULL, 's'},
  {"count", no_argument, NULL, 'c'}, 
  {"file", no_argument, NULL, 'f'}, 
  {"verbose", no_argument, NULL, 'v'}, 
  {"help", no_argument, NULL, 'h'},
  {0, 0, 0, 0}  
};

static int test_dma(char *devicename, uint32_t addr, uint32_t size, uint32_t count, char *filename);

static void usage(const char* name)
{
  int i = 0;
  printf("%s\n\n", name);
  printf("usage: %s [OPTIONS]\n\n", name);
  printf("Read using SGDMA and write output to output.bin\n\n", name);

  printf("  -%c (--%s) device (defaults to ./lancero_sgdma)\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) address of the start address on the Avalon bus\n", long_opts[i].val, long_opts[i].name); i++;      
  printf("  -%c (--%s) size of a single transfer\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) number of transfers\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) filename to read/write the data of the transfers\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) be more verbose during test\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) print usage help and exit\n", long_opts[i].val, long_opts[i].name); i++;
}

static uint32_t getopt_integer(char *optarg)
{
  int rc;
  uint32_t value;
  rc = sscanf(optarg, "0x%x", &value);
  if (rc <= 0)
    rc = sscanf(optarg, "%ul", &value);
  printf("sscanf() = %d, value = 0x%08x\n", rc, (unsigned int)value);
  return value;
}

int main(int argc, char* argv[])
{
  int cmd_opt;
  char *device = "./lancero_sgdma";
  uint32_t address = 0;
  uint32_t size = 32;
  uint32_t count = 1;
  char *filename = NULL;

  while ((cmd_opt = getopt_long(argc, argv, "vhc:f:d:a:s:", long_opts, NULL)) != -1)
  {
    switch (cmd_opt)
    {
      case 0:
        /* long option */        
        break;
      case 'v':
        verbosity++;
        break;
      /* device node name */
      case 'd':
	printf("'%s'\n", optarg);
        device = strdup(optarg);
        break;                
      /* RAM address on the Avalon bus in bytes */
      case 'a':
        address = getopt_integer(optarg);
        break;
      /* RAM size in bytes */
      case 's':
        size = getopt_integer(optarg);
        break;
      /* count */
      case 'c':
        count = getopt_integer(optarg);
        break;
      /* count */
      case 'f':
        filename = strdup(optarg);
        break;
      /* print usage help and exit */
      case 'h':
      default:
        usage(argv[0]);
        exit(0);
        break;
    }
  }
  printf("device = %s, address = 0x%08x, size = 0x%08x, count = %u\n", device, address, size, count);
  test_dma(device, address, size, count, filename);

}

static int test_dma(char *devicename, uint32_t addr, uint32_t size, uint32_t count, char *filename)
{
  int rc;
  char *buffer = NULL;
  posix_memalign((void **)&buffer, 32, size);
  assert(buffer);

  int file_fd = -1;
  int fpga_fd = open(devicename, O_RDWR | O_NONBLOCK);
  assert(fpga_fd >= 0);
  
  /* create file to write data to */
  if (filename) {
    file_fd = open(filename, O_RDWR | O_CREAT | O_TRUNC | O_SYNC, 0666);
    assert(file_fd >= 0);
  }

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
  close(fpga_fd);
  if (file_fd >=0) {
    close(file_fd);
  }
  free(buffer);
}
