/* Read/Write SGDMA Tester for the Lancero Scatter-Gather Reference Design
 *
 * The Lancero IP core reference design has read and write data generators
 * that act as a source and destination for SGDMA streaming. They generate
 * and expect sequentially incrementing data patterns.
 *
 * This application starts one thread for each data generator in the system
 * and sets up a series of randomly-sized SGDMA requests.
 *
 * For SGDMA to the FPGA, it initializes the sequence in a buffer and after
 * DMA verifies that the FPGA received the sequence correctly.
 *
 * Similarly for SGDMA from the FPGA, it verifies the sequence on the host.
 *
 * @author Leon Woestenberg <leon@sidebranch.com>
 */

#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

/* number of read threads, each thread drives a write tester module in the
 * FPGA SOPC system at addresses 0x1000 + n * 0x2000, connected to WDMA */
#define NUMR 1
/* number of write threads, each thread drives a read tester module in the
 * FPGA SOPC system at addresses 0x2000 + n * 0x2000, connected to RDMA */
#define NUMW 1

/* the maximum number of bytes to read/write from/to the write/read testers */ 
#define MAX_SIZE_BYTES (64 * 1024 * 1024)

#define SGDMA_ALIGNMENT (32)

/* seed for the pseudo random generator */
uint32_t seed = 1;

/* pseudo random number generator used to generate random read/write sizes */
static uint32_t xorshift_random(uint32_t x)
{
  x ^= (x << 3);
  x ^= (x >> 13);
  x ^= (x << 7);
  return x;
}

/* number of bytes for the head and tail guard buffers, which are adjacent
 * to the user-space DMA buffers and which verify against buffer underruns
 * and overruns.
 */
#define GUARD_SIZE_BYTES (32768)

/* tests the SGDMA write engine by reading from the end-point */
void *read_work(void *id)
{
  unsigned long long read_total = 0;
  unsigned int n = (unsigned int)id & 3;
  printf("Start read_work %d.\n", n);

  int fd = open("lancero_sgdma", O_RDWR);
  assert(fd >= 0);

  while (1)
  {

  int fd_user = open("lancero_user", O_RDWR | O_SYNC);
  assert(fd_user >= 0);
  uint32_t buffer;
  int rc = pread(fd_user, &buffer, 4, 0x1000 + n * 0x2000);
  printf("write tester ID = 0x%x\n", buffer);
  /* reset write datagenerator (CLR bit) */
  buffer = 0x00000001UL;
#if defined(__BIG_ENDIAN__)
  buffer = __builtin_bswap32(buffer);
#endif
  rc = pwrite(fd_user, &buffer, 4, 0x1000 + n * 0x2000 + 0x4);

  uint32_t expected = 0;
  /* make sure expected does not run over */
  while (expected < (0x3fffffffUL - MAX_SIZE_BYTES))
  {
    /* choose a random size */
    seed = xorshift_random(seed);
    uint32_t words = (seed & (MAX_SIZE_BYTES / SGDMA_ALIGNMENT - 1) & 0xfffffff0UL) + 1;
    uint32_t size = words * SGDMA_ALIGNMENT;

    char *buf_in;
    rc = posix_memalign(&buf_in, SGDMA_ALIGNMENT, size + 2 * GUARD_SIZE_BYTES);
    assert(rc == 0);
    /* assert allocation */
    assert(buf_in);
    /* assert alignment */
    assert(((int)buf_in & (SGDMA_ALIGNMENT - 1)) == 0);
    char *buf_dma = buf_in + GUARD_SIZE_BYTES;
    /* write header guard */
    memset(buf_in, 0xaa, GUARD_SIZE_BYTES);
    /* write tailer guard */
    memset(buf_in + GUARD_SIZE_BYTES + size, 0xbb, GUARD_SIZE_BYTES);
#if 0
    printf("%d: pread(size = %u)\n", n, size);

    printf("expected = %08lu\n", expected);
    printf("%d: pread(size=%d @ 0x%lx)\n", n, size, n * 0x40000000L);
#endif
    int rc = pread(fd , buf_dma, size, n * 0x40000000L);
    if (rc != size) printf("%d: pread(size=%d @ 0x%lx)=%d\n", n, size, n * 0x40000000L, rc);
    assert(rc == size);

    uint32_t first = *(uint32_t *)buf_dma;
    uint32_t last = *(uint32_t *)(buf_dma + size - 4);
#if 0
    printf("buf_dma[0] = %08lu\n", first);
    printf("buf_dma[%u] = %08lu\n", size - 4, last);
#endif
#if 1
    printf("%d: datagenerator counter progress: %u\n", n, (last - first + 1) * 4);
#endif
    for (int i = 0; i < GUARD_SIZE_BYTES; i += 1)
    {
      uint8_t value = *(buf_in + i);
      if (value != 0xaa)
      {
         printf("Data mismatch @%u in header guard.\n", i);
         abort();
      }
    }
    for (int i = 0; i < GUARD_SIZE_BYTES; i += 1)
    {
      uint8_t value = *(buf_in + GUARD_SIZE_BYTES + size + i);
      if (value != 0xbb)
      {
         printf("Data mismatch @%u in tailer guard.\n", GUARD_SIZE_BYTES + size + i);
         abort();
      }
    }
    for (int i = 0; i < size; i += 4)
    {
      uint32_t value = *(uint32_t *)(buf_in + GUARD_SIZE_BYTES + i);
#if defined(__BIG_ENDIAN__)
      value = __builtin_bswap32(value);
#endif
      if (value != expected)
      {
        printf("Data mismatch @buf_dma[%u]==%lu, expected==%lu\n", i, value, expected);
        abort();
      }
      expected++;
    }
    free(buf_in);
    printf(" read() of size %d OK\n", size);
  }
  }
  close(fd);
}

/* tests the SGDMA read engine by writing to the end-point */
void *write_work(void* id)
{
  unsigned int n = (unsigned int)id & 3;
  printf("Start write_work %d.\n", n);

  int fd_events = open("lancero_events", O_RDWR | O_SYNC);
  assert(fd_events >= 0);

  int fd_user = open("lancero_user", O_RDWR | O_SYNC);
  assert(fd_user >= 0);

  int fd = open("lancero_sgdma", O_RDWR);
  assert(fd >= 0);

  printf("Start writing.\n");

  while (1)
  {

  uint32_t buffer;
  int rc = pread(fd_user, &buffer, 4, 0x2000 + n * 0x2000);
  printf("read tester ID = 0x%x\n", buffer);
  /* reset read datagenerator (IE and CLR bits) */
  buffer = 0x00000003UL;
#if defined(__BIG_ENDIAN__)
    buffer = __builtin_bswap32(buffer);
#endif
    rc = pwrite(fd_user, &buffer, 4, 0x2000 + n * 0x2000 + 0x4);
    assert(rc == 4);

    /* the incremental value the tester expects us to write */
    uint32_t expected = 0;
    /* make sure expected does not run over */
    while (expected < (0x3fffffffUL - MAX_SIZE_BYTES))
    {
      /* choose a random transfer size */
      seed = xorshift_random(seed);
      uint32_t words = (seed & (MAX_SIZE_BYTES / SGDMA_ALIGNMENT - 1) & 0xfffffff0UL) + 1;
      uint32_t size = words * SGDMA_ALIGNMENT;

//      size = 1024 *4096 ;

      char *buf;
      rc = posix_memalign(&buf, SGDMA_ALIGNMENT, size);
      assert(rc == 0);
      /* assert allocation */
      assert(buf);
      /* assert alignment */
      assert(((int)buf & (SGDMA_ALIGNMENT - 1)) == 0);

#if 1
      printf("expected = %08lu\n", expected);
#endif
      /* fill buffer with expected sequence */
      for (int i = 0; i < size; i += 4)
      {
#if defined(__BIG_ENDIAN__)
        *(uint32_t *)(buf + i) = __builtin_bswap32(expected);
#  warning BIG_ENDIAN
#else
        *(uint32_t *)(buf + i) = expected;
#endif
        expected++;
       }
#if 0 /* inject non-sequential failure */
      *(uint32_t *)(buf + size - 4) += 1;
#endif
      uint32_t first = *(uint32_t *)buf;
      uint32_t last = *(uint32_t *)(buf + size - 4);
#if 1
      printf("buf[0] = %08lu\n", first);
      printf("buf[%u] = %08lu\n", size - 4, last);
#endif
      printf("%d: pwrite(size = %u)\n", n, size);
      int rc = pwrite(fd , buf, size, n * 0x40000000);

      assert(rc == size);
      free(buf); buf = NULL;

      uint32_t event = 0;
      rc = read(fd_events, &event, 4);
      assert(rc == 4);
#if 1
      printf("event = 0x%08x\n", event);
#endif
      if (event & 0x00000001UL)
      {
        uint32_t status;
        uint32_t counter;
        rc = pread(fd_user, &counter, 4, 0x2000 + n * 0x2000 + 0x8);
        assert(rc == 4);
        rc = pread(fd_user, &counter, 4, 0x2000 + n * 0x2000 + 0xc);
        assert(rc == 4);
        printf("Write data mismatch, status = 0x%08x, counter = 0x%08x\n", status, counter);
        /* disable interrupt of read tester */
        buffer = 0x00000001UL;
        rc = pwrite(fd_user, &buffer, 4, 0x2000 + n * 0x2000 + 0x4);
        if (rc != 4) perror("pwrite");
        assert(rc == 4);
        abort();
      }
      printf("write() of size %d OK\n", size);
    }
    /* disable interrupt of read tester */
    buffer = 0x00000001UL;
    rc = pwrite(fd_user, &buffer, 4, 0x2000 + n * 0x2000 + 0x4);
    assert(rc == 4);
  }
  close(fd);
  close(fd_user);
  close(fd_events);
}

int main(int argc, char* argv[])
{
  /* reads from the device, this exercises the device write(!) engine */
  pthread_t reader[NUMR];
  /* writes to the device, this exercises the device read(!) engine */
  pthread_t writer[NUMW];

  for (int i = 0; i < NUMR; i += 1)
  {
    pthread_create(&reader[i], NULL, read_work, (void*)i);
    printf("Spawned reader %d\n", i);
  }
  for (int i = 0; i < NUMW; i += 1)
  {
    pthread_create(&writer[i], NULL, write_work, (void*)i);
    printf("Spawned writer %d\n", i);
  }
  sleep(3);
  for (int i = 0; i < NUMR; i += 1)
  {
    printf("Joining reader %d\n", i);
    pthread_join(reader[i], NULL);
    printf("Joined reader %d\n", i);
  }
  for (int i = 0; i < NUMW; i += 1)
  {
    printf("Joining writer %d\n", i);
    pthread_join(writer[i], NULL);
    printf("Joined writer %d\n", i);
  }
}
