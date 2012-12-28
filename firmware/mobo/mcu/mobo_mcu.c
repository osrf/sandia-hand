/*  Software License Agreement (Apache License)
 *
 *  Copyright 2012 Open Source Robotics Foundation
 *  Author: Morgan Quigley
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "common_sam3x/sam3x.h"
#include "common_sam3x/console.h"
#include "fpga_spi.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>

void systick_vector()
{
}

void main()
{
  console_init();
  fpga_spi_init();
  printf("hello, world!\r\n");
  for (int i = 0; i < 32; i++)
  {
    //printf("%d\r\n", i);
    printf("fpga register %d: 0x%04x\r\n", i, fpga_spi_txrx(0x80 | i, 0));
    for (volatile int j = 0; j < 100000; j++) { }
  }
  printf("entering main loop\r\n");
  while (1)
  {
  }
}

///////////////////////////////////////////////////////////////////////////
// libc stub nonsense

extern int _end;
extern caddr_t _sbrk(int incr);

extern caddr_t _sbrk(int incr)
{
  static unsigned char *heap = NULL ;
  unsigned char *prev_heap ;
  if ( heap == NULL )
    heap = (unsigned char *)&_end ;
  prev_heap = heap;
  heap += incr ;
  return (caddr_t) prev_heap ;
}
extern int _kill(int pid, int sig) { return -1; }
extern void _exit(int status) { }
int _getpid() { return 1; }
extern int _write(int fd, const void *buf, size_t count)
{
  //io_led(true);
  console_send_block((uint8_t *)buf, count);
  //io_led(false);
  return count;
}
extern int _close(int fd) { return -1; }
int _fstat(int fd, struct stat *st)
{
  st->st_mode = S_IFCHR;
    return 0;
}
int _isatty(int fd) { return 1; }
off_t _lseek(int fd, off_t offset, int whence) { return 0; }
ssize_t _read(int fd, void *buf, size_t count) { return 0; }

struct __FILE { int handle; };
FILE __stdout;
FILE __stderr;
int fputc(int ch, FILE *f)
{
  //console_send_string("fputc\r\n");
  return 0;
}
void _ttywrch(int ch)
{
  //console_send_string("ttywrch\r\n");
}


