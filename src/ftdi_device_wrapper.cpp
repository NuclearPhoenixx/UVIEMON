/*
	================================================
	uviemon: free(TM) replacement for grmon

	Wrapper functions to manage compatability between
	the new C++ I/O functions and the C DSU library.
	================================================
*/

#include "ftdi_device_wrapper.hpp"

#include "uviemon.hpp" // To get the "device" object, not a great way to do it...
#include <iostream>

using namespace std;

static FTDIDevice &handle = device; // Declare the existing FTDIDevice instance as a global variable

extern "C" void pr_err(const char *output)
{
	cerr << "[!] DSU ERROR: " << output << endl; // Replacement for now for the sake of simplicity
}

extern "C" void iowrite16be(uint16_t data, void *addr)
{
	handle.iowrite16(*(uint32_t *)addr, data);
}

extern "C" void iowrite32be(uint32_t data, void *addr)
{
	handle.iowrite32(*(uint32_t *)addr, data);
}

extern "C" uint16_t ioread16be(void *addr)
{
	return handle.ioread16(*(uint32_t *)addr);
}

extern "C" uint32_t ioread32be(void *addr)
{
	return handle.ioread32(*(uint32_t *)addr);
}
