/*
	================================================
	uviemon: free(TM) replacement for grmon

	Wrapper functions to manage compatability between
	the new C++ I/O functions and the C DSU library.
	================================================
*/

#ifndef FTDI_DEVICE_WRAPPER_HPP
#define FTDI_DEVICE_WRAPPER_HPP

#include <cstdint> // Use datatypes "uint32_t", ...

#ifdef __cplusplus
extern "C"
{
#endif

	void pr_err(const char *output);

	void iowrite16be(uint16_t data, void *addr); // SEGFAULTS!
	void iowrite32be(uint32_t data, void *addr); // SEGFAULTS!
	uint16_t ioread16be(void *addr);			 // SEGFAULTS!
	uint32_t ioread32be(void *addr);			 // SEGFAULTS!

#ifdef __cplusplus
}
#endif

#endif /* FTDI_DEVICE_WRAPPER_HPP */
