#include <stdio.h>
#include <stdint.h>
#include <pigpio.h>

int main(void)
{
  int handle, ret;
  ret = gpioInitialise();
  if (ret == PI_INIT_FAILED) {
    printf("gpioInitialise failed\n");
    return 1;
  }

  handle = i2cOpen(1, 0x68, 0);
  if (handle) {
    printf("i2cOpen: %d\n", handle);
    return 1;
  }

  ret = i2cWriteByteData(handle, 0x6B, 0x01);
  if (ret) {
    printf("i2cWriteByteData: %d\n", ret);
    return 1;
  }

  ret = i2cWriteByteData(handle, 0x1C, 0x2 << 3);
  if (ret) {
    printf("i2cWriteByteData: %d\n", ret);
    return 1;
  }

  int16_t z_val;
  z_val = i2cReadWordData(handle, 0x3F);
  switch(z_val) {
  case PI_BAD_HANDLE:
  case PI_BAD_PARAM:
  case PI_I2C_READ_FAILED:
    printf("Something went wrong :(\n");
    break;
  default:
    break;
  }
  printf("i2cReadWordData: %x, %d\n", z_val, z_val);
  z_val = ((z_val & 0xFF00) >> 8 | ((z_val & 0x00FF) << 8));
  printf("i2cReadWordData: %x, %d\n", z_val, z_val);
  
  gpioTerminate();
  return 0;
}
