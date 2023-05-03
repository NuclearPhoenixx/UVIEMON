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

	void iowrite16be(uint16_t data, uint32_t addr);
	void iowrite32be(uint32_t data, uint32_t addr);
	uint16_t ioread16be(uint32_t addr);
	uint32_t ioread32be(uint32_t addr);

#ifdef __cplusplus
}
#endif

#endif /* FTDI_DEVICE_WRAPPER_HPP */
