/*
 * Progmem.h
 *
 *  Created on: Feb 11, 2015
 *      Author: adrien
 */

#ifndef PROGMEM_H_
#define PROGMEM_H_

#include <string.h>
#include <stdint.h>
#include <avr/pgmspace.h>



typedef char prog_char;

#define PSTR(s) s
#undef PROGMEM
#define PROGMEM __attribute__(())
/**

static inline int strcasecmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcasecmp(str1, pstr);
}

static inline int strcmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcmp(str1, pstr);
}

static inline size_t strlen_P(const prog_char_t *pstr)
{
    return strlen(pstr);
}

static inline void *memcpy_P(void *dest, const prog_char_t *src, size_t n)
{
    return memcpy(dest, src, n);
}



static inline char *strncpy_P(char *buffer, const prog_char_t *pstr, size_t buffer_size)
{
    return strncpy(buffer, pstr, buffer_size);
}

static inline size_t strnlen_P(const prog_char_t *str, size_t size)
{
	return strnlen(str, size);
}

static inline int strncmp_P(const char *str1, const prog_char_t *str2, size_t n)
{
	return strncmp(str1, str2, n);
}


// read something the size of a byte
static inline uint8_t pgm_read_byte(const void *s) {
	return *(const uint8_t *)s;
}

// read something the size of a byte, far version
static inline uint8_t pgm_read_byte_far(const void *s) {
	return *(const uint8_t *)s;
}

// read something the size of a word
static inline uint16_t pgm_read_word(const void *s) {
	return *(const uint16_t *)s;
}

// read something the size of a dword
static inline uint32_t pgm_read_dword(const void *s) {
	return *(const uint32_t *)s;
}

// read something the size of a float
static inline float pgm_read_float(const void *s) {
	return *(const float *)s;
}

// read something the size of a pointer. This makes the menu code more
// portable
static inline uintptr_t pgm_read_pointer(const void *s) {
	return *(const uintptr_t *)s;
}
*/

// prog_char_t is used as a wrapper type for prog_char, which is
// a character stored in flash. By using this wrapper type we can
// auto-detect at compile time if a call to a string function is using
// a flash-stored string or not
typedef struct {
    char c;
} prog_char_t;

#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

#undef PSTR
/* need to define prog_char in avr-gcc 4.7 */
#if __AVR__ && __GNUC__ == 4 && __GNUC_MINOR__ > 6
typedef char prog_char;
#endif
/* Need const type for progmem - new for avr-gcc 4.6 */
#if __AVR__ && __GNUC__ == 4 && __GNUC_MINOR__ > 5
#define PSTR(s) (__extension__({static const prog_char __c[] PROGMEM = (s); \
                                  (const prog_char_t *)&__c[0]; }))
#else
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); \
                                  (prog_char_t *)&__c[0]; }))
#endif

/*
static inline int strcasecmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcasecmp_P(str1, (const prog_char *)pstr);
}

static inline int strcmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcmp_P(str1, (const prog_char *)pstr);
}

static inline size_t strlen_P(const prog_char_t *pstr)
{
    return strlen_P((const prog_char *)pstr);
}

static inline void *memcpy_P(void *dest, const prog_char_t *src, size_t n)
{
    return memcpy_P(dest, (const prog_char *)src, n);
}


static inline char *strncpy_P(char *buffer, const prog_char_t *pstr, size_t buffer_size)
{
    return strncpy_P(buffer, (const prog_char *)pstr, buffer_size);
}


// read something the size of a pointer. This makes the menu code more
// portable
static inline uintptr_t pgm_read_pointer(const void *s)
{
    if (sizeof(uintptr_t) == sizeof(uint16_t)) {
        return (uintptr_t)pgm_read_word(s);
    } else {
        union {
            uintptr_t p;
            uint8_t a[sizeof(uintptr_t)];
        } u;
        uint8_t        i;
        for (i=0; i< sizeof(uintptr_t); i++) {
            u.a[i] = pgm_read_byte(i + (const prog_char *)s);
        }
        return u.p;
    }
}


// strlcat_P() in AVR libc seems to be broken
size_t strlcat_P(char *d, const prog_char_t *s, size_t bufsize)
{
    size_t len1 = strlen(d);
    size_t len2 = strlen_P(s);
    size_t ret  = len1 + len2;

    if (len1+len2 >= bufsize) {
        if (bufsize < (len1+1)) {
            return ret;
        }
        len2 = bufsize - (len1+1);
    }
    if (len2 > 0) {
        memcpy_P(d+len1, s, len2);
        d[len1+len2] = 0;
    }
    return ret;
}
*/

#endif /* PROGMEM_H_ */
