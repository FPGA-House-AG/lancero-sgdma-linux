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

#define LOG_SIZE 100

struct access_log {
	int align_mode;
	int access_write;
	int access_width;
	uint32_t offset;
	uint32_t value;
};

static struct option const long_opts[] =
{
  {"device", required_argument, NULL, 'd'},
  {"address", required_argument, NULL, 'a'},
  {"size", required_argument, NULL, 's'},
  {"number", required_argument, NULL, 'n'},
  {"seed", required_argument, NULL, 'r'},
  {"readback", no_argument, NULL, 'b'},
  {"accesses", required_argument, NULL, 'c'},
  {"verbose", no_argument, NULL, 'v'}, 
  {"help", no_argument, NULL, 'h'},
  {0, 0, 0, 0}  
};

static void usage(const char* name)
{
  int i = 0;
  printf("%s\n\n", name);
  printf("usage: %s [OPTIONS]\n\n", name);

  printf("  -%c (--%s) device\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) address of the RAM on the Avalon bus\n", long_opts[i].val, long_opts[i].name); i++;      
  printf("  -%c (--%s) size of the RAM on the Avalon bus (in bytes, default: 32)\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) number of accesses before test completes (default: 1)\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) pseudo-random start seed number (default: 1)\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) read back and compare using 32-bit words after each write\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) allowed access size/alignment/offset (in bytes) tripplets, 1=4/4/0 2=2/4/0, 3=2/2/1, 4=4/4/0, 5=1/1/1 6=1/2/2 7=1/1/3\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) be more verbose during test\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) print usage help and exit\n", long_opts[i].val, long_opts[i].name); i++;
}

static int test_ram_vectors(char *device, uint32_t seed, uint32_t addr, uint32_t size, int number);

static const char *align_name[] = {
  "32-bit word",
  "16-bit halfword on offset 0",
  "16-bit halfword on offset 1",
  "8-bit byte on offset 0",		
  "8-bit byte on offset 1",		
  "8-bit byte on offset 2",
  "8-bit byte on offset 3",
};

static uint32_t getopt_integer(char *optarg)
{
  int rc;
  uint32_t value;
  rc = sscanf(optarg, "0x%x", &value);
  if (rc == 0)
    rc = sscanf(optarg, "%d", &value);
  //printf("sscanf() = %d, value = %u\n", rc, value);
  return value;
}

int main(int argc, char* argv[])
{
  int cmd_opt;  
  char *device = "/dev/lancero";
  uint32_t address = 0;
  uint32_t size = 32;  
  uint32_t number = 1;  
  uint32_t seed = 1;  

  while ((cmd_opt = getopt_long(argc, argv, "vhd:bc:a:s:n:r:", long_opts, NULL)) != -1)
  {
    switch (cmd_opt)
    {
      case 0:
        /* long option */        
        break;
      case 'v':
        verbosity++;
        break;
      case 'b':
        read_back = 1;
        break;
      case 'c':
        allowed_accesses = getopt_integer(optarg);
        break;
      /* device node name */
      case 'd':
	printf("optarg = %s\n", optarg);
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
      /* number of accesses */
      case 'n':
        number = getopt_integer(optarg);
        break;
      /* random seed */
      case 'r':
        seed = getopt_integer(optarg);
        break;
      /* print usage help and exit */
      case 'h':
      default:
        usage(argv[0]);
        exit(0);
        break;
    }
  }
  printf("device = %s, address = 0x%08x, size = 0x%08x\n", device, address, size);
  for (int i = 0; i < allowed_accesses; i++)
    printf("%s\n", align_name[i]);
  int rc = test_ram_vectors(device, seed, address, size, number);
  printf(rc?"\nFAILED\n":"\nOK\n");
  return rc;
}

static uint32_t xorshift_random(uint32_t x)
{
	x ^= (x << 3);
	x ^= (x >> 13);
	x ^= (x << 7);
	return x;
}

static const int align_offset[] = { 0, 0, 2, 0, 1, 2 ,3 };
static const int access_size[] = { 4, 2, 2, 1, 1, 1, 1 };

static void playback(struct access_log *log, int n)
{
	int j = 0;
	while (j < LOG_SIZE)
	{
		int i = (n + j + 1) % LOG_SIZE;
		printf("%8d: %s to address 0x%08x, %s", n - LOG_SIZE + j + 1,
			align_name[log[i % LOG_SIZE].align_mode],
			log[i % LOG_SIZE].offset,
			log[i % LOG_SIZE].access_write?"write":"read");
		if (log[i % LOG_SIZE].access_write)
			if (log[i % LOG_SIZE].align_mode == 0)
				printf(", value 0x%08x", log[i].value);
			else if ((log[i % LOG_SIZE].align_mode == 1) || (log[i % LOG_SIZE].align_mode == 2))
				printf(", value 0x%04x", log[i % LOG_SIZE].value);
			else
				printf(", value 0x%02x", log[i % LOG_SIZE].value);
		printf("\n");
		j++;
	}
};

static int test_ram_vectors(char *devicename, uint32_t seed, uint32_t addr, uint32_t size, int number)
{
	/* map shadow file */
	int fd = open("shadow.bin", O_RDWR | O_CREAT | O_TRUNC | O_SYNC, 0666);
	assert(fd >= 0);
	int rc = ftruncate(fd, size);
	assert(rc == 0);
	uint8_t *shadow_buffer;
	shadow_buffer = (uint8_t *)mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	assert(shadow_buffer != (uint8_t *)MAP_FAILED);
	printf("shadow_buffer = %p\n", shadow_buffer);

	int consecutive_writes = 0, max_writes = 0;

#ifdef SELFTEST
	uint8_t *mapped_buffer = (uint8_t *)calloc(size, 1);
	if (mapped_buffer == NULL)
	{
		munmap(shadow_buffer, size);
		close(fd);
		return -1;
	}
#else
	/* map fpga device */
	int fdd = open(devicename, O_RDWR | O_SYNC);
	assert(fdd >= 0);
	/* map the ram at offset 'addr' and with length 'size' */
	uint8_t *mapped_buffer = (uint8_t *)mmap(NULL, (size < 4096) ? size: 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fdd, 0);
	assert(shadow_buffer != (uint8_t *)MAP_FAILED);
	mapped_buffer += addr;
	printf("mapped_buffer = 0x%0x\n", mapped_buffer);
#endif
	/* copy ram contents to shadow buffer using 32-bit word reads */
	int i = 0;
	while (i < size)
	{
		/* copy fpga values to shadow buffer */
		uint32_t fpga_value = *(uint32_t *)(mapped_buffer + i);
		*(uint32_t *)(shadow_buffer + i) = fpga_value;
		i += 4;
	}

	struct access_log *log;
	log = calloc(sizeof(struct access_log), LOG_SIZE);
	assert(log);

	i = 0;
	/* number of test vectors */
	while (i < number)
	{
		int align_mode, access_write, access_width;
		uint32_t offset;
		uint32_t value;
		seed = xorshift_random(seed);
		/* access type in range [0, 7)
		 * 32-bit word 
		 * 16-bit offset 0 against word-aligned address  
		 * 16-bit offset 1 against word-aligned address
		 *  8-bit byte offset 0 against word-aligned address		
		 *  8-bit byte offset 1 against word-aligned address		
		 *  8-bit byte offset 2 against word-aligned address		
		 *  8-bit byte offset 3 against word-aligned address
		 */		
		align_mode = seed & ((1 << 3/*bits*/) - 1);
		/* limit to one of seven alignment modes */
		align_mode %= allowed_accesses;
		/* obtain the width of the access (1, 2 or 4) */
		access_width = access_size[align_mode];

		/* 0 = read, 1 = write */
		access_write = (seed >> 3) & ((1 << 1) - 1);

		/* seed random generator for address offset */
		seed = xorshift_random(seed);

		/* offset in [0, size) */
		offset = seed % size;
		/* make 32-bit word aligned */
		offset &= ~3;
		offset += align_offset[align_mode];

		/* seed random generator again for value */
		seed = xorshift_random(seed);
		value = seed;

		/* shift so that all unused high bytes are zeroed */
		value >>= ((4 - access_width) * 8);

#if 1
		log[i % LOG_SIZE].align_mode = align_mode;
		log[i % LOG_SIZE].access_write = access_write;
		log[i % LOG_SIZE].access_width = access_width;
		log[i % LOG_SIZE].offset = offset;
		log[i % LOG_SIZE].value = value;
#endif

		if (verbosity)
		{
			printf("%s to address 0x%08x, %s", align_name[align_mode],
				offset, access_write?"write":"read");
			if (access_write)
				if (align_mode == 0)
					printf(", value 0x%08x", value);
				else if ((align_mode == 1) || (align_mode == 2))
					printf(", value 0x%04x", value);
				else
					printf(", value 0x%02x", value);
			printf("\n");
		}
		/* write access? */
		if (access_write) 
		{
//			assert(align_mode == 0);
			if (align_mode == 0)
			{
				*(uint32_t *)(shadow_buffer + offset) = (uint32_t)value;
				*(uint32_t *)(mapped_buffer + offset) = (uint32_t)value;
			}
			else if ((align_mode == 1) || (align_mode == 2))
			{
				*(uint16_t *)(shadow_buffer + offset) = (uint16_t)value;
				*(uint16_t *)(mapped_buffer + offset) = (uint16_t)value;
			}
			else
			{
				*(uint8_t *)(shadow_buffer + offset) = (uint8_t)value;
				*(uint8_t *)(mapped_buffer + offset) = (uint8_t)value;
			}
			/* 32-bit read access with compare after each write access? */
			if (read_back)
			{
				offset &= ~3;
				uint32_t fpga_value = *(uint32_t *)(mapped_buffer + offset);
				uint32_t shadow_value = *(uint32_t *)(shadow_buffer + offset);
				if (fpga_value != shadow_value) {
					printf("#%8d: Mismatch on address 0x%08x, fpga: 0x%08x, shadow: 0x%08x during read-back.\n",
						i, offset, fpga_value, shadow_value);
					playback(log, i);
					break;
				}
				assert(fpga_value == shadow_value);
			}
		}
		/* read access */
		else
		{
			/* 32-bit access? */
			if (align_mode == 0)
			{
				uint32_t fpga_value = *(uint32_t *)(mapped_buffer + offset);
				uint32_t shadow_value = *(uint32_t *)(shadow_buffer + offset);
				if (fpga_value != shadow_value) {
					printf("#%8d: Mismatch on address 0x%08x, fpga: 0x%08x, shadow: 0x%08x\n",
						i, offset, fpga_value, shadow_value);
					playback(log, i);
					break;
				}
				assert(fpga_value == shadow_value);
			}
			/* 16-bit access? */
			else if ((align_mode == 1) || (align_mode == 2))
			{
				uint16_t fpga_value = *(uint16_t *)(mapped_buffer + offset);
				uint16_t shadow_value = *(uint16_t *)(shadow_buffer + offset);
				if (fpga_value != shadow_value) {
					printf("#%8d: Mismatch on address 0x%08x, fpga: 0x%04x, shadow: 0x%04x\n",
						i, offset, fpga_value, shadow_value);
					playback(log, i);
					break;
				}
				assert(fpga_value == shadow_value);
			}
			/* 8-bit access? */
			else
			{
				uint8_t fpga_value = *(uint8_t *)(mapped_buffer + offset);
				uint8_t shadow_value = *(uint8_t *)(shadow_buffer + offset);
				if (fpga_value != shadow_value) {
					printf("#%8d: Mismatch on address 0x%08x, fpga: 0x%02x, shadow: 0x%02x\n",
						i, offset, fpga_value, shadow_value);
					playback(log, i);
					break;
				}
				assert(fpga_value == shadow_value);
			}
		}

		assert((offset % access_width) == 0);
		i++;
		/* read access? */
		if (access_write == 0) {
			/* number of earlier consecutive writes is max? */
			if (consecutive_writes > max_writes) {
				max_writes = consecutive_writes;
			}
			consecutive_writes = 0;
		} else consecutive_writes++;
	} /*while*/

	printf("max_writes = %d\n", max_writes);
	printf("consecutive_writes = %d\n", consecutive_writes);

	/* verify complete memory using 32-bit word reads */
	i = 0;
	while (i < size)
	{
		uint32_t fpga_value = *(uint32_t *)(mapped_buffer + i);
		uint32_t shadow_value = *(uint32_t *)(shadow_buffer + i);
		if (fpga_value != shadow_value) {
			printf("Mismatch on address 0x%08x, fpga: 0x%08x, shadow: 0x%08x during buffer compare.\n", i, fpga_value, shadow_value);
			printf("\nFAILED\n");
			return -1;
		}
		assert(fpga_value == shadow_value);
		i += 4;
	}
#ifdef SELFTEST
	free(mapped_buffer);
#else
	mapped_buffer -= addr;
	munmap(mapped_buffer, (size < 4096) ? size: 4096);
	close(fdd);
#endif
	munmap(shadow_buffer, size);
	close(fd);
	return 0;
}
