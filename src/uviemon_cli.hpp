/*
	============================================
	uviemon: free(TM) replacement for grmon

	Functions used in the command line interface
	of uviemon. All these functions corrrespond
	to their command in the console to be called
	by the user.
	============================================
*/

#ifndef UVIEMON_CLI_HPP
#define UVIEMON_CLI_HPP

#include "ftdi_device.hpp"
#include <string>

std::string _hexToString(DWORD *data, size_t size);
std::string _hexToString(WORD *data, size_t size);
std::string _hexToString(BYTE *data, size_t size);

void wmem(FTDIDevice &handle, DWORD addr, DWORD data);
void wmemh(FTDIDevice &handle, DWORD addr, WORD data);
void wmemb(FTDIDevice &handle, DWORD addr, BYTE data);

void mem(FTDIDevice &handle, DWORD startAddr, DWORD length = 16);
void memh(FTDIDevice &handle, DWORD startAddr, DWORD length = 32);
void memb(FTDIDevice &handle, DWORD startAddr, DWORD length = 64);

void wash(FTDIDevice &handle, WORD size = 16, DWORD addr = 0x40000000, DWORD c = 0); // Doesn't work correctly for >1024B due to ioread32...
void load(FTDIDevice &handle, std::string &path);									 // Doesn't work correctly for >1024B due to ioread32...
void verify(FTDIDevice &handle, std::string &path);
// void run(FTDIDevice &handle); // TODO: Run Uploaded Executable

#endif /* UVIEMON_CLI_HPP */
