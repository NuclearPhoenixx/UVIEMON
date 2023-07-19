/*
	==========================================
	uviemon: free(TM) replacement for grmon

	I/O functions per (FTDI) device handle.

	Uses the standard FT2232H linux drivers to
	communicate to the processor via JTAG.

	Can be used to instantiate multiple JTAG devices
	at the same time, in the same program.
	==========================================
*/

#ifndef FTDI_DEVICE_HPP
#define FTDI_DEVICE_HPP

#include "lib/ftdi/ftd2xx.h"

const unsigned int CODE_ADDR_COMM = 0x2; // address/command register opcode, 35-bit length
const DWORD CODE_DATA = 0x3;			 // data register opcode, 33-bit length

const BYTE RW_DWORD = 0b0000010; // 10 for 32-bit DWORD
const BYTE RW_WORD = 0b0000001;	 // 01 for 16-bit WORD
const BYTE RW_BYTE = 0b00000000; // 00 for 8-bit BYTE

class FTDIDevice
{
private:
	FT_HANDLE _ftHandle;
	DWORD _deviceIndex;

	FT_STATUS _initMPSSEMode();
	FT_STATUS _resetJTAGStateMachine();

	void _initCore1();
	void _initCore2();

public:
	FTDIDevice();  // Constructor
	~FTDIDevice(); // Destructor

	DWORD getDevicesCount();
	void getDeviceList(); // Has CLI Output!

	FT_STATUS open(DWORD deviceIndex = 0); // Default: Use Device index 0

	/*
		Scans and checks
	*/
	DWORD readIDCODE();
	BYTE getJTAGCount();
	BYTE scanIRLength();
	BYTE scanDRLength(BYTE opcode);
	void scanInstructionCodes(BYTE maxIRLength); // Has CLI Output!

	/*
		DSU operations for runnning programs
	*/
	BYTE runCPU(BYTE cpuID); // 0 or 1 for either core
	void reset(BYTE cpuID = 0);	 // Resets CPU

	/*
		Memory RW operations
	*/
	BYTE ioread8(DWORD addr);
	WORD ioread16(DWORD addr);
	DWORD ioread32(DWORD addr);

	void iowrite8(DWORD addr, BYTE data);
	void iowrite16(DWORD addr, WORD data);
	void iowrite32(DWORD addr, DWORD data);

	// Sequential RW w/ optional progress output
	void ioread32raw(DWORD startAddr, DWORD *data, WORD size); // Extremely  (!) slow, it's actually faster to write data...
	void iowrite32raw(DWORD startAddr, DWORD *data, WORD size);
	void ioread32(DWORD startAddr, DWORD *data, WORD size, bool progress = false);
	void iowrite32(DWORD startAddr, DWORD *data, WORD size, bool progress = false);
};

#endif /* FTDI_DEVICE_HPP */
