#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

enum {
  LCD_RST = 0,
  LCD_CS = 3,
  LCD_RS  = 1,

  LOW = 0,
  HIGH = 1,

  INPUT = 0,
  OUTPUT = 1,
};

struct pio_cfg {
  volatile uint32_t cfg[4];
  volatile uint32_t dat;
  volatile uint32_t drv[2];
  volatile uint32_t pul[2];
}* pa;

int spi;
uint8_t tx[1];
uint8_t rx[1];
struct spi_ioc_transfer tr = {
  .len = 1,
  .speed_hz = 10000000,
  .delay_usecs = 0,
  .bits_per_word = 8,
  .cs_change = 0,
  .tx_nbits = 8,
  .rx_nbits = 8,
};

void pinMode(int gpio_pin, int mode) {
  volatile uint32_t* cfg = (gpio_pin < 8) ? &pa->cfg[0] :
      (gpio_pin < 16) ? &pa->cfg[1] : &pa->cfg[2];
  int bit = (gpio_pin & 7) * 4;
  *cfg &= ~(0xf << bit);
  if (mode == OUTPUT)
    *cfg |= (0x1 << bit);
  volatile uint32_t* pul = (gpio_pin < 16) ? &pa->pul[0] : &pa->pul[1];
  *pul &= ~(3 << ((gpio_pin & 15) * 2));
}

void digitalWrite(int gpio_pin, int val) {
  uint32_t value = pa->dat;
  value &= ~(1 << gpio_pin);
  if (val)
    value |= (1 << gpio_pin);
  pa->dat = value;
}

uint16_t digitalRead(int gpio_pin) {
  uint32_t value = pa->dat;
  return (value >> gpio_pin) & 1;
}

void delay(int t) {
  usleep(t * 1000);
}

void delayMicroseconds(int t) {
  usleep(t);
}

void lcdInit()
{
  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);
  pinMode(LCD_RS, OUTPUT);
  digitalWrite(LCD_RS, HIGH);
  pinMode(LCD_RST, OUTPUT);
  digitalWrite(LCD_RST, HIGH);
}

void lcdReset()
{
  digitalWrite(LCD_RST, LOW);
  delay(20);
  digitalWrite(LCD_RST, HIGH);
  delay(20);
  digitalWrite(LCD_CS, LOW);
}

uint8_t lcdWriteData(uint8_t data)
{
  digitalWrite(LCD_RS, HIGH);
  tx[0] = data;
  ioctl(spi, SPI_IOC_MESSAGE(1), &tr);
  return rx[0];
}

uint8_t lcdWriteCommand(uint8_t command)
{
  digitalWrite(LCD_RS, LOW);
  tx[0] = command;
  ioctl(spi, SPI_IOC_MESSAGE(1), &tr);
  return rx[0];
}

uint8_t lcdReadData8()
{
  digitalWrite(LCD_RS, HIGH);
  tx[0] = 0;
  ioctl(spi, SPI_IOC_MESSAGE(1), &tr);
  return rx[0];
}

void readReg(uint8_t reg, uint8_t n, const char *msg)
{
  uint8_t val8;
  lcdReset();
  lcdWriteCommand(0xB0);     //Command Access Protect
  lcdWriteData(0x00);        //looks wrong

  val8 = lcdWriteCommand(reg);
  printf("reg(0x%04x) %02x", reg, val8);
  n--;
  while (n--) {
    val8 = lcdReadData8();
    printf(" %02x", val8);
  }
  printf("\t %s\n", msg);
}

void setup()
{
  puts("Read LCD Registers");

  lcdInit();

  lcdReset();
  //for (uint16_t i = 0; i < 256; i++) readReg(i, 7, "f.k"); 

  readReg(0x00, 2, "ID: ILI9320, ILI9325, ILI9335, ...");
  readReg(0x04, 4, "Manufacturer ID");
  readReg(0x09, 5, "Status Register");
  readReg(0x0A, 2, "Get Power Mode");
  readReg(0x0C, 2, "Get Pixel Format");
  readReg(0x61, 2, "RDID1 HX8347-G");
  readReg(0x62, 2, "RDID2 HX8347-G");
  readReg(0x63, 2, "RDID3 HX8347-G");
  readReg(0x64, 2, "RDID1 HX8347-A");
  readReg(0x65, 2, "RDID2 HX8347-A");
  readReg(0x66, 2, "RDID3 HX8347-A");
  readReg(0x67, 2, "RDID Himax HX8347-A");
  readReg(0x70, 2, "Panel Himax HX8347-A");
  readReg(0xA1, 5, "RD_DDB SSD1963");
  readReg(0xB0, 2, "RGB Interface Signal Control");
  readReg(0xB4, 2, "Inversion Control");
  readReg(0xB6, 5, "Display Control");
  readReg(0xB7, 2, "Entry Mode Set");
  readReg(0xBF, 6, "ILI9481, HX8357-B");
  readReg(0xC0, 9, "Panel Control");
  readReg(0xC8, 13, "GAMMA");
  readReg(0xCC, 2, "Panel Control");
  readReg(0xD0, 3, "Power Control");
  readReg(0xD2, 5, "NVM Read");
  readReg(0xD3, 4, "ILI9341, ILI9488");
  readReg(0xD4, 4, "Novatek ID");
  readReg(0xDA, 2, "RDID1");
  readReg(0xDB, 2, "RDID2");
  readReg(0xDC, 2, "RDID3");
  readReg(0xE0, 16, "GAMMA-P");
  readReg(0xE1, 16, "GAMMA-N");
  readReg(0xEF, 6, "ILI9327");
  readReg(0xF2, 12, "Adjust Control 2");
  readReg(0xF6, 4, "Interface Control");
}

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

  spi = open("/dev/spidev1.0", O_RDWR);
  tr.tx_buf = (__u64)tx;
  tr.rx_buf = (__u64)rx;

  setup();
  return 0;
}
