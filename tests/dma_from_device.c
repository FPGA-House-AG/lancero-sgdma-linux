#define _BSD_SOURCE
#define _XOPEN_SOURCE 500
#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

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
  printf("Read using SGDMA, optionally writing data to file.\n\n");

  printf("  -%c (--%s) device (defaults to ./lancero_sgdma)\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) address of the start address on the Avalon bus\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) size of a single transfer\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) number of transfers\n", long_opts[i].val, long_opts[i].name); i++;
  printf("  -%c (--%s) filename to write the data of the transfers to\n", long_opts[i].val, long_opts[i].name); i++;
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

/* subtracts t2 from t1, the result is in t1
 * t1 and t2 should be already normalized, i.e. nsec in [0, 1000000000)
 */
static void timespec_sub(struct timespec *t1, const struct timespec *t2)
{
  assert(t1->tv_nsec >= 0);
  assert(t1->tv_nsec < 1000000000);
  assert(t2->tv_nsec >= 0);
  assert(t2->tv_nsec < 1000000000);
  t1->tv_sec -= t2->tv_sec;
  t1->tv_nsec -= t2->tv_nsec;
  if (t1->tv_nsec >= 1000000000)
  {
    t1->tv_sec++;
    t1->tv_nsec -= 1000000000;
  }
  else if (t1->tv_nsec < 0)
  {
    t1->tv_sec--;
    t1->tv_nsec += 1000000000;
  }
}

static long long total_ns = 0;
static long long largest_ns = 0;
static long long largest_difference_ns = 0;
static long long total_num = 0;

#define NUM_TIMINGS_MAX 6000
static long timings[NUM_TIMINGS_MAX];

static int add_timing(long ns)
{
  /* store timing sample */
  timings[total_num] = ns;
  /* remember largest timing sample */
  if (ns > largest_ns) largest_ns = ns;
  /* accumulate samples */
  total_ns += ns;
  total_num += 1;
}

/* calculate mean of samples */
static long long timing_mean(void)
{
  return total_ns / total_num;
}

static long long timing_variance(long long mean)
{
  long long result_variance = 0;
  long long total_squared_diffs_ns = 0;

  largest_difference_ns = 0;

  assert(total_num >= 1);
  /* subtract mean from each sample, and square the result, all in-place */
  /* @NOTE: this modifies the timings[], so take and remember mean first! */
  for (int i = 0; i < total_num; i++)
  {
     /* calculate difference between sample and mean */
    long long difference_ns = (long long)timings[i] - (long long)mean;

    /* remember largest difference */
    if (difference_ns > largest_difference_ns) largest_difference_ns = difference_ns;

    /* accumulate squared sample difference to mean */
    total_squared_diffs_ns += (difference_ns * difference_ns);
  }
  result_variance = total_squared_diffs_ns / total_num;
  return result_variance;
}

static double timing_deviation(long long variance)
{
  return sqrt(variance);
}

static void timing_print(void)
{
  /* calculate and remember mean value */
  long long mean = timing_mean();
  long long variance = timing_variance(mean);
  double deviation = timing_deviation(variance);
  printf("mean = %lld ns, largest = %lld ns, largest_difference = %lld, variance = %lld ns, deviation = %f ns\n",
    mean, largest_ns, largest_difference_ns, variance, deviation);
}

static int test_dma(char *devicename, uint32_t addr, uint32_t size, uint32_t count, char *filename)
{
  int rc;
  char *buffer = NULL;
  struct timespec ts_start, ts_end;
  posix_memalign((void **)&buffer, 32, size);
  assert(buffer);

  assert(count <= NUM_TIMINGS_MAX);

  int file_fd = -1;
  int fpga_fd = open(devicename, O_RDWR | O_NONBLOCK);
  assert(fpga_fd >= 0);

  /* create file to write data to */
  if (filename) {
    file_fd = open(filename, O_RDWR | O_CREAT | O_TRUNC | O_SYNC, 0666);
    assert(file_fd >= 0);
  }

  while (count--) {

    rc = clock_gettime(CLOCK_MONOTONIC, &ts_start);

#if 0
    /* select Avalon MM address */
    off_t off = lseek(fpga_fd, addr, SEEK_SET);
    /* read data from Avalon MM into buffer using SGDMA */
    rc = read(fpga_fd, buffer, size);
#else
    /* do the above in one system call */
    rc = pread(fpga_fd, buffer, size, addr);
#endif

    assert(rc == size);

    rc = clock_gettime(CLOCK_MONOTONIC, &ts_end);
    /* subtract the start time from the end time */
    timespec_sub(&ts_end, &ts_start);
    assert(ts_end.tv_sec == 0);
    add_timing(ts_end.tv_nsec);

    /* file argument given? */
    if (file_fd >= 0) {
      /* write buffer to file */
      rc = write(file_fd, buffer, size);
      assert(rc == size);
    }
  }

  timing_print();

  /* display passed time, a bit less accurate but side-effects are accounted for */
  printf("CLOCK_MONOTONIC reports %ld.%09ld seconds (total) for %d bytes\n", ts_end.tv_sec, ts_end.tv_nsec, size);

  close(fpga_fd);
  if (file_fd >=0) {
    close(file_fd);
  }
  free(buffer);
}
