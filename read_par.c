#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

enum {
  A0 = 0,   // LCD_RD
  A1 = 1,   // LCD_WR
  A2 = 10,  // LCD_RS
  A3 = 11,  // LCD_CS
  A4 = 12,  // LCD_RST

  LOW = 0,
  HIGH = 1,

  INPUT = 0,
  OUTPUT = 1,
};

int gpio_map[13] = {
  19,  // LCD_RD
  18,  // LCD_WR
  15,  // LCD_D2
   3,  // LCD_D3
   0,  // LCD_D4
   1,  // LCD_D5
   6,  // LCD_D6
  11,  // LCD_D7
  14,  // LCD_D0
  16,  // LCD_D1
   2,  // LCD_RS
  13,  // LCD_CS
  10,  // LCD_RST
};

void nil_i(int i) {}
void printh(uint8_t n) {
  printf("%X", n);
}

struct {
  void (*begin)(int);
  int (*println)(const char*);
  int (*print)(const char*, ...);
  void (*printhex)(uint8_t);
} Serial = {
  .begin = nil_i,
  .println = puts,
  .print = printf,
  .printhex = printh,
};

void readReg(uint16_t, uint8_t, const char*);
void lcdInit();
void lcdReset();
void lcdWrite8(uint16_t);
uint16_t lcdRead8();
void lcdSetWriteDir();
void lcdSetReadDir();
void lcdWriteData(uint16_t);
void lcdWriteCommand(uint16_t);
uint8_t lcdReadData8();
uint16_t lcdReadData16();
void lcdWriteRegister(uint16_t, uint16_t);

struct pio_cfg {
  volatile uint32_t cfg[4];
  volatile uint32_t dat;
  volatile uint32_t drv[2];
  volatile uint32_t pul[2];
}* pa;

void pinMode(int pin, int mode) {
  int gpio_pin = gpio_map[pin];
  //printf("pinMode: %d(%d), %d\n", pin, gpio_pin, mode);
  volatile uint32_t* cfg = (gpio_pin < 8) ? &pa->cfg[0] :
      (gpio_pin < 16) ? &pa->cfg[1] : &pa->cfg[2];
  int bit = (gpio_pin & 7) * 4;
  *cfg &= ~(0xf << bit);
  if (mode == OUTPUT)
    *cfg |= (0x1 << bit);
  volatile uint32_t* pul = (gpio_pin < 16) ? &pa->pul[0] : &pa->pul[1];
  *pul &= ~(3 << ((gpio_pin & 15) * 2));

}

void digitalWrite(int pin, int val) {
  int gpio_pin = gpio_map[pin];
  uint32_t value = pa->dat;
  value &= ~(1 << gpio_pin);
  if (val)
    value |= (1 << gpio_pin);
  pa->dat = value;
}

uint16_t digitalRead(int pin) {
  int gpio_pin = gpio_map[pin];
  uint32_t value = pa->dat;
  return (value >> gpio_pin) & 1;
}

void delay(int t) {
  usleep(t * 1000);
}

void delayMicroseconds(int t) {
  usleep(t);
}

#include "LCD_ID_readreg.ino"

#define GPIO_PA_BASE 0x01c20000
#define GPIO_PA_OFFSET 0x0800

int main(int argc, char** argv) {
  int mem = open("/dev/mem", O_RDWR | O_SYNC);
  if (mem < 0) {
    perror("open /dev/mem");
    return -1;
  }

  int prot = PROT_READ | PROT_WRITE;
  size_t size = sysconf(_SC_PAGE_SIZE);
  char *pa_base = mmap(NULL, size, prot, MAP_SHARED, mem, GPIO_PA_BASE);
  pa = (struct pio_cfg*)&pa_base[GPIO_PA_OFFSET];

  setup();
  return 0;
}
