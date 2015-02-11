/*
 * UARTDriver.h
 *
 *  Created on: Feb 11, 2015
 *      Author: adrien
 */

#ifndef UARTDRIVER_H_
#define UARTDRIVER_H_

#include "Common.h"
#include "arch/AVR/UART/Progmem.h"
#include <limits.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>

#include <avr/pgmspace.h>
#include <avr/interrupt.h>


/**
 * AVRUARTDriver is an implementation of UARTDriver for the AVR.
 * It will be a thin wrapper on FastSerial.
 */

class AVRUARTDriver : public Print {
public:
	AVRUARTDriver(
			const uint8_t portNumber, volatile uint8_t *ubrrh,
			volatile uint8_t *ubrrl, volatile uint8_t *ucsra,
			volatile uint8_t *ucsrb, const uint8_t u2x,
			const uint8_t portEnableBits, const uint8_t portTxBits);

	/* Implementations of UARTDriver virtual methods */
	void begin(uint32_t b) { begin(b, 0, 0); }
	void begin(uint32_t b, uint16_t rxS, uint16_t txS);
	void end();
	void flush();
	bool is_initialized() { return _initialized; }

	void set_blocking_writes(bool blocking) {
		_nonblocking_writes = !blocking;
	}

	bool tx_pending() {
		return (_txBuffer->head != _txBuffer->tail);
	}

	/* Implementations of Stream virtual methods */
	int16_t available();
	int16_t txspace();
	int16_t read();

	/* Implementations of Print virtual methods */
	size_t write(uint8_t c);
	size_t write(const uint8_t *buffer, size_t size);

	void print_P(const prog_char_t *s);
	void println_P(const prog_char_t *s);
	void printf(const char *s, ...)
	__attribute__ ((format(__printf__, 2, 3)));
	void _printf_P(const prog_char *s, ...)
	__attribute__ ((format(__printf__, 2, 3)));

	/// Transmit/receive buffer descriptor.
	///
	/// Public so the interrupt handlers can see it
	struct Buffer {
		volatile uint8_t head, tail;	///< head and tail pointers
		uint8_t mask;					///< buffer size mask for pointer wrap
		uint8_t *bytes;					///< pointer to allocated buffer
	};
private:
	/* Instance Variables */
	bool _initialized;

	// register accessors
	volatile uint8_t * const _ubrrh;
	volatile uint8_t * const _ubrrl;
	volatile uint8_t * const _ucsra;
	volatile uint8_t * const _ucsrb;

	// register magic numbers
	const uint8_t	_u2x;
	const uint8_t	_portEnableBits;		///< rx, tx and rx interrupt enables
	const uint8_t	_portTxBits;			///< tx data and completion interrupt enables

	// ring buffers
	Buffer			* const _rxBuffer;
	Buffer			* const _txBuffer;
	bool 			_open;

	// whether writes to the port should block waiting
	// for enough space to appear
	bool			_nonblocking_writes;

	/* Class Variables */

	/// Allocates a buffer of the given size
	///
	/// @param	buffer		The buffer descriptor for which the buffer will
	///						will be allocated.
	/// @param	size		The desired buffer size.
	/// @returns			True if the buffer was allocated successfully.
	///
	static bool _allocBuffer(Buffer *buffer, uint16_t size);

	/// Frees the allocated buffer in a descriptor
	///
	/// @param	buffer		The descriptor whose buffer should be freed.
	///
	static void _freeBuffer(Buffer *buffer);

	/// default receive buffer size
	static const uint16_t _default_rx_buffer_size = 4;

	/// default transmit buffer size
	static const uint16_t _default_tx_buffer_size = 16;

	/// maxium tx/rx buffer size
	static const uint16_t _max_buffer_size = 256;
};

extern AVRUARTDriver::Buffer __AVRUARTDriver__rxBuffer[];
extern AVRUARTDriver::Buffer __AVRUARTDriver__txBuffer[];

/// Generic Rx/Tx vectors for a serial port - needs to know magic numbers
///
#define AVRUARTDriverHandler(_PORT, _RXVECTOR, _TXVECTOR, _UDR, _UCSRB, _TXBITS) \
		ISR(_RXVECTOR, ISR_BLOCK)                                               \
		{                                                                       \
	uint8_t c;                                                      \
	uint8_t i;                                                     \
	\
	/* read the byte as quickly as possible */                      \
		c = _UDR;                                                       \
		/* work out where the head will go next */                      \
		i = (__AVRUARTDriver__rxBuffer[_PORT].head + 1) & __AVRUARTDriver__rxBuffer[_PORT].mask; \
		/* decide whether we have space for another byte */             \
		if (i != __AVRUARTDriver__rxBuffer[_PORT].tail) {                  \
			/* we do, move the head */                              \
		__AVRUARTDriver__rxBuffer[_PORT].bytes[__AVRUARTDriver__rxBuffer[_PORT].head] = c; \
		__AVRUARTDriver__rxBuffer[_PORT].head = i;                 \
		}                                                               \
		}                                                                       \
		ISR(_TXVECTOR, ISR_BLOCK)                                               \
		{                                                                       \
			/* if there is another character to send */                     \
		if (__AVRUARTDriver__txBuffer[_PORT].tail != __AVRUARTDriver__txBuffer[_PORT].head) { \
			_UDR = __AVRUARTDriver__txBuffer[_PORT].bytes[__AVRUARTDriver__txBuffer[_PORT].tail]; \
			/* increment the tail */                                \
		__AVRUARTDriver__txBuffer[_PORT].tail =                    \
		(__AVRUARTDriver__txBuffer[_PORT].tail + 1) & __AVRUARTDriver__txBuffer[_PORT].mask; \
		} else {                                                        \
			/* there are no more bytes to send, disable the interrupt */ \
		if (__AVRUARTDriver__txBuffer[_PORT].head == __AVRUARTDriver__txBuffer[_PORT].tail) \
		_UCSRB &= ~_TXBITS;                             \
		}                                                               \
		}                                                                       \
		struct hack

//
// Portability; convert various older sets of defines for U(S)ART0 up
// to match the definitions for the 1280 and later devices.
//
#if !defined(USART0_RX_vect)
# if defined(USART_RX_vect)
#  define USART0_RX_vect        USART_RX_vect
#  define USART0_UDRE_vect      USART_UDRE_vect
# elif defined(UART0_RX_vect)
#  define USART0_RX_vect        UART0_RX_vect
#  define USART0_UDRE_vect      UART0_UDRE_vect
# endif
#endif

#if !defined(USART1_RX_vect)
# if defined(UART1_RX_vect)
#  define USART1_RX_vect        UART1_RX_vect
#  define USART1_UDRE_vect      UART1_UDRE_vect
# endif
#endif

#if !defined(UDR0)
# if defined(UDR)
#  define UDR0                  UDR
#  define UBRR0H                UBRRH
#  define UBRR0L                UBRRL
#  define UCSR0A                UCSRA
#  define UCSR0B                UCSRB
#  define U2X0                  U2X
#  define RXEN0                 RXEN
#  define TXEN0                 TXEN
#  define RXCIE0                RXCIE
#  define UDRIE0                UDRIE
# endif
#endif

///
/// Macro defining a AVRUARTDriver port instance.
///
#define AVRUARTDriverInstance(_name, _num)                              \
		AVRUARTDriver _name(_num,                                           \
				&UBRR##_num##H,                                \
				&UBRR##_num##L,                                \
				&UCSR##_num##A,                                \
				&UCSR##_num##B,                                \
				U2X##_num,                                     \
				(_BV(RXEN##_num) |  _BV(TXEN##_num) | _BV(RXCIE##_num)), \
				(_BV(UDRIE##_num)));

#define AVRUARTDriverISRs(_num)                                         \
		AVRUARTDriverHandler(_num,                                          \
				USART##_num##_RX_vect,                        \
				USART##_num##_UDRE_vect,                      \
				UDR##_num,                                    \
				UCSR##_num##B,                                \
				_BV(UDRIE##_num))





#define FS_MAX_PORTS 4
AVRUARTDriver::Buffer __AVRUARTDriver__rxBuffer[FS_MAX_PORTS];
AVRUARTDriver::Buffer __AVRUARTDriver__txBuffer[FS_MAX_PORTS];

AVRUARTDriver::AVRUARTDriver(
		const uint8_t portNumber, volatile uint8_t *ubrrh,
		volatile uint8_t *ubrrl, volatile uint8_t *ucsra,
		volatile uint8_t *ucsrb, const uint8_t u2x,
		const uint8_t portEnableBits, const uint8_t portTxBits) :
					_ubrrh(ubrrh),
					_ubrrl(ubrrl),
					_ucsra(ucsra),
					_ucsrb(ucsrb),
					_u2x(u2x),
					_portEnableBits(portEnableBits),
					_portTxBits(portTxBits),
					_rxBuffer(&__AVRUARTDriver__rxBuffer[portNumber]),
					_txBuffer(&__AVRUARTDriver__txBuffer[portNumber])
{
	_initialized = true;
	begin(57600);
}
/* UARTDriver method implementations */

void AVRUARTDriver::begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) {
	uint16_t ubrr;
	bool use_u2x = true;
	bool need_allocate = true;

	// if we are currently open...
	if (_open) {
		// If the caller wants to preserve the buffer sizing, work out what
		// it currently is...
		if (0 == rxSpace)
			rxSpace = _rxBuffer->mask + 1;
		if (0 == txSpace)
			txSpace = _txBuffer->mask + 1;

		if (rxSpace == (_rxBuffer->mask + 1U) &&
				txSpace == (_txBuffer->mask + 1U)) {
			// avoid re-allocating the buffers if possible
			need_allocate = false;
			*_ucsrb &= ~(_portEnableBits | _portTxBits);
		} else {
			// close the port in its current configuration, clears _open
			end();
		}
	}

	if (need_allocate) {
		// allocate buffers
		if (!_allocBuffer(_rxBuffer, rxSpace ? : _default_rx_buffer_size)
				|| !_allocBuffer(_txBuffer, txSpace ? : _default_tx_buffer_size)) {
			end();
			return; // couldn't allocate buffers - fatal
		}
	}

	// reset buffer pointers
	_txBuffer->head = _txBuffer->tail = 0;
	_rxBuffer->head = _rxBuffer->tail = 0;

	// mark the port as open
	_open = true;

	// If the user has supplied a new baud rate, compute the new UBRR value.
	if (baud > 0) {
#if F_CPU == 16000000UL
		// hardcoded exception for compatibility with the bootloader shipped
		// with the Duemilanove and previous boards and the firmware on the 8U2
		// on the Uno and Mega 2560.
		if (baud == 57600)
			use_u2x = false;
#endif

		if (use_u2x) {
			*_ucsra = 1 << _u2x;
			ubrr = (F_CPU / 4 / baud - 1) / 2;
		} else {
			*_ucsra = 0;
			ubrr = (F_CPU / 8 / baud - 1) / 2;
		}

		*_ubrrh = ubrr >> 8;
		*_ubrrl = ubrr;
	}

	*_ucsrb |= _portEnableBits;

}

void AVRUARTDriver::end() {
	*_ucsrb &= ~(_portEnableBits | _portTxBits);

	_freeBuffer(_rxBuffer);
	_freeBuffer(_txBuffer);
	_open = false;
}

int16_t AVRUARTDriver::available(void) {
	if (!_open)
		return (-1);
	return ((_rxBuffer->head - _rxBuffer->tail) & _rxBuffer->mask);
}



int16_t AVRUARTDriver::txspace(void) {
	if (!_open)
		return (-1);
	return ((_txBuffer->mask+1) - ((_txBuffer->head - _txBuffer->tail) & _txBuffer->mask));
}

int16_t AVRUARTDriver::read(void) {
	uint8_t c;

	// if the head and tail are equal, the buffer is empty
	if (!_open || (_rxBuffer->head == _rxBuffer->tail))
		return (-1);

	// pull character from tail
	c = _rxBuffer->bytes[_rxBuffer->tail];
	_rxBuffer->tail = (_rxBuffer->tail + 1) & _rxBuffer->mask;

	return (c);
}

void AVRUARTDriver::flush(void) {
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of _rxBuffer->head but before writing
	// the value to _rxBuffer->tail; the previous value of head
	// may be written to tail, making it appear as if the buffer
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of head but before writing
	// the value to tail; the previous value of rx_buffer_head
	// may be written to tail, making it appear as if the buffer
	// were full, not empty.
	_rxBuffer->head = _rxBuffer->tail;

	// don't reverse this or there may be problems if the TX interrupt
	// occurs after reading the value of _txBuffer->tail but before writing
	// the value to _txBuffer->head.
	_txBuffer->tail = _txBuffer->head;
}

size_t AVRUARTDriver::write(uint8_t c) {
	uint8_t i;

	if (!_open) // drop bytes if not open
		return 0;

	// wait for room in the tx buffer
	i = (_txBuffer->head + 1) & _txBuffer->mask;

	// if the port is set into non-blocking mode, then drop the byte
	// if there isn't enough room for it in the transmit buffer
	if (_nonblocking_writes && i == _txBuffer->tail) {
		return 0;
	}

	while (i == _txBuffer->tail)
		;

	// add byte to the buffer
	_txBuffer->bytes[_txBuffer->head] = c;
	_txBuffer->head = i;

	// enable the data-ready interrupt, as it may be off if the buffer is empty
	*_ucsrb |= _portTxBits;

	// return number of bytes written (always 1)
	return 1;
}

/*
  write size bytes to the write buffer
 */
size_t AVRUARTDriver::write(const uint8_t *buffer, size_t size)
{
	if (!_open) {
		return 0;
	}

	if (!_nonblocking_writes) {
		/*
          use the per-byte delay loop in write() above for blocking writes
		 */
		size_t ret = 0;
		while (size--) {
			if (write(*buffer++) != 1) break;
			ret++;
		}
		return ret;
	}

	int16_t space = txspace();
	if (space <= 0) {
		return 0;
	}
	if (size > (size_t)space) {
		// throw away remainder if too much data
		size = space;
	}
	if (_txBuffer->tail > _txBuffer->head) {
		// perform as single memcpy
		memcpy(&_txBuffer->bytes[_txBuffer->head], buffer, size);
		_txBuffer->head = (_txBuffer->head + size) & _txBuffer->mask;
		// enable the data-ready interrupt, as it may be off if the buffer is empty
		*_ucsrb |= _portTxBits;
		return size;
	}

	// perform as two memcpy calls
	uint16_t n = (_txBuffer->mask+1) - _txBuffer->head;
	if (n > size) n = size;
	memcpy(&_txBuffer->bytes[_txBuffer->head], buffer, n);
	_txBuffer->head = (_txBuffer->head + n) & _txBuffer->mask;
	buffer += n;
	n = size - n;
	if (n > 0) {
		memcpy(&_txBuffer->bytes[0], buffer, n);
		_txBuffer->head = (_txBuffer->head + n) & _txBuffer->mask;
	}

	// enable the data-ready interrupt, as it may be off if the buffer is empty
	*_ucsrb |= _portTxBits;
	return size;
}

// Buffer management ///////////////////////////////////////////////////////////


bool AVRUARTDriver::_allocBuffer(Buffer *buffer, uint16_t size)
{
	uint8_t mask;
	uint8_t	 shift;

	// init buffer state
	buffer->head = buffer->tail = 0;

	// Compute the power of 2 greater or equal to the requested buffer size
	// and then a mask to simplify wrapping operations.  Using __builtin_clz
	// would seem to make sense, but it uses a 256(!) byte table.
	// Note that we ignore requests for more than BUFFER_MAX space.
	for (shift = 1; (1U << shift) < min(_max_buffer_size, size); shift++)
		;
	mask = (1U << shift) - 1;

	// If the descriptor already has a buffer allocated we need to take
	// care of it.
	if (buffer->bytes) {

		// If the allocated buffer is already the correct size then
		// we have nothing to do
		if (buffer->mask == mask)
			return true;

		// Dispose of the old buffer.
		free(buffer->bytes);
	}
	buffer->mask = mask;

	// allocate memory for the buffer - if this fails, we fail.
	buffer->bytes = (uint8_t *) malloc(buffer->mask + (size_t)1);

	return (buffer->bytes != NULL);
}

void AVRUARTDriver::_freeBuffer(Buffer *buffer)
{
	buffer->head = buffer->tail = 0;
	buffer->mask = 0;
	if (NULL != buffer->bytes) {
		free(buffer->bytes);
		buffer->bytes = NULL;
	}
}


void AVRUARTDriver::print_P(const prog_char_t *s)
{
	char    c;
	while ('\0' != (c = pgm_read_byte((const prog_char *)s++)))
		write(c);
}

void AVRUARTDriver::println_P(const prog_char_t *s)
{
	print_P(s);
	println();
}

void AVRUARTDriver::printf(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}




#endif /* UARTDRIVER_H_ */
