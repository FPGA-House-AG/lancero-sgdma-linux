#include <assert.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

int main(void)
{
  int fd = open("/alt", O_RDWR | O_SYNC);
  printf("fd = %d\n", fd);
  assert(fd >= 0);
  uint32_t *addr = mmap(NULL, 16 * 1024 * 1024, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  assert(addr != (uint32_t *)MAP_FAILED);

  printf("the target bridge base address is mapped at 0x%p\n", addr);
  uint32_t *addr32 = addr;
  uint16_t *addr16 = (uint16_t *)addr;
  uint8_t *addr8 = (uint16_t *)addr;

  addr32 = addr + (0x800000) / 4;
  addr16 = addr + (0x800000) / 4;
  addr8 = addr + (0x800000) / 4;
  printf("addr32 = 0x%p\n", addr32);
  printf("addr16 = 0x%p\n", addr16);
  printf("addr8 = 0x%p\n\n", addr8);
  *addr32 = 0x11223344;
  uint32_t data = *addr32;
  printf("address 0x%p reads data 0x%08x\n", addr32, (unsigned int)data);
  uint8_t data8 = *addr8;
  printf("address 0x%p reads data 0x%02x\n", addr8, (unsigned int)data8);
  printf("\n");
#if 0
  addr32 = addr + 0x100;
  for (int i = 0; i < 3; i++)
  {
    uint32_t value = 0x1000 + i;
    value *= 2;
    printf("writing 0x%08x to 0x%p\n", value, addr32);
    *addr32 = value;
     addr32++;
  }
#endif

#if 1
  addr32 = addr /*+ 0x100*/;
  printf(" addr32 = 0x%p\n", addr32);
  for (int i = 0; i < 4; i++)
  {
    uint32_t data = *addr32;
    printf("address 0x%p reads data 0x%08x\n", addr32, (unsigned int)data);
    addr32++;
  }
#endif
  munmap(addr, 16 * 1024 * 1024);
  close(fd);
}

