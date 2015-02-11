/*
 * Print.h
 *
 *  Created on: Feb 11, 2015
 *      Author: adrien
 */

#ifndef PRINT_H_
#define PRINT_H_


#include <inttypes.h>
#include <string.h>

/**
 * This is the Arduino (v1.0) Print class, with some changes:
 * - Removed methods for class String or _FlashStringHelper
 * - printFloat takes a float, not a double. (double === float on AVR, but
 *   not on other platforms)
 */

enum {
    BASE_DEFAULT = 0,
    BASE_BIN     = 2,
    BASE_OCT     = 8,
    BASE_DEC     = 10,
    BASE_HEX     = 16
};


class Print {
  private:
    size_t printNumber(unsigned long, uint8_t);
    size_t printFloat(float, uint8_t);
  public:
    Print() {}

    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;

    size_t write(const char *str) { return write((const uint8_t *)str, strlen(str)); }
  public:
    size_t print(const char[]);
    size_t print(char);
    size_t print(unsigned char, int = BASE_DEC);
    size_t print(int, int = BASE_DEC);
    size_t print(unsigned int, int = BASE_DEC);
    size_t print(long, int = BASE_DEC);
    size_t print(unsigned long, int = BASE_DEC);
    size_t print(float , int = 2);
    size_t print(double , int = 2);

    size_t println(const char[]);
    size_t println(char);
    size_t println(unsigned char, int = BASE_DEC);
    size_t println(int, int = BASE_DEC);
    size_t println(unsigned int, int = BASE_DEC);
    size_t println(long, int = BASE_DEC);
    size_t println(unsigned long, int = BASE_DEC);
    size_t println(float , int = 2);
    size_t println(double , int = 2);
    size_t println(void);

};

size_t Print::print(const char str[])
{
  return write(str);
}

size_t Print::print(char c)
{
  return write(c);
}

size_t Print::print(unsigned char b, int base)
{
  return print((unsigned long) b, base);
}

size_t Print::print(int n, int base)
{
  return print((long) n, base);
}

size_t Print::print(unsigned int n, int base)
{
  return print((unsigned long) n, base);
}

size_t Print::print(long n, int base)
{
  if (base == 0) {
    return write(n);
  } else if (base == 10) {
    if (n < 0) {
      int t = print('-');
      n = -n;
      return printNumber(n, 10) + t;
    }
    return printNumber(n, 10);
  } else {
    return printNumber(n, base);
  }
}

size_t Print::print(unsigned long n, int base)
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

size_t Print::print(float n, int digits)
{
  return printFloat(n, digits);
}

// the compiler promotes to double if we do arithmetic in the
// argument, but we only actually want float precision, so just wrap
// it with a double method
size_t Print::print(double n, int digits)
{
	return print((float)n, digits);
}

size_t Print::println(void)
{
  size_t n = print('\r');
  n += print('\n');
  return n;
}

size_t Print::println(const char c[])
{
  size_t n = print(c);
  n += println();
  return n;
}

size_t Print::println(char c)
{
  size_t n = print(c);
  n += println();
  return n;
}

size_t Print::println(unsigned char b, int base)
{
  size_t n = print(b, base);
  n += println();
  return n;
}

size_t Print::println(int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(unsigned int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(long num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(unsigned long num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(float num, int digits)
{
  size_t n = print(num, digits);
  n += println();
  return n;
}

// the compiler promotes to double if we do arithmetic in the
// argument, but we only actually want float precision, so just wrap
// it with a double method
size_t Print::println(double num, int digits)
{
  return println((float)num, digits);
}

// Private Methods /////////////////////////////////////////////////////////////

size_t Print::printNumber(unsigned long n, uint8_t base) {
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  return write(str);
}

size_t Print::printFloat(float number, uint8_t digits)
{
  size_t n = 0;

  // Handle negative numbers
  if (number < 0.0f)
  {
     n += print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  float rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0f;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  float remainder = number - (float )int_part;
  n += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    n += print(".");
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0f;
    int toPrint = int(remainder);
    n += print(toPrint);
    remainder -= toPrint;
  }

  return n;
}


#endif /* PRINT_H_ */
