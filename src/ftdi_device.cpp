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

#include "ftdi_device.hpp"

#include <iostream>
#include <unistd.h> // Unix lib for sleep
#include <cmath>	// For std::ceil in ioread/write32()

#include "leon3_dsu.h" // Interface to the GR712 debug support unit

using namespace std;

FTDIDevice::FTDIDevice()
{
	// cout << "Hello World!" << endl;
}

FTDIDevice::~FTDIDevice()
{
	// Reset device before closing handle, good practice
	FT_SetBitMode(_ftHandle, 0x0, 0x00);
	FT_ResetDevice(_ftHandle);

	// Close device
	FT_Close(_ftHandle);
	cout << "Goodbye." << endl;
}

DWORD FTDIDevice::getDevicesCount()
{
	FT_STATUS ftStatus;
	DWORD numDevs;

	ftStatus = FT_CreateDeviceInfoList(&numDevs);
	if (ftStatus != FT_OK)
	{
		cerr << "Failed to grab number of attached devices" << endl;
		return ftStatus;
	}
	return numDevs;
}

void FTDIDevice::getDeviceList()
{
	DWORD numDevs = getDevicesCount();

	cout << "Number of devices: " << numDevs << endl;
	cout << endl;

	for (DWORD i = 0; i < numDevs; i++)
	{
		DWORD flags;
		DWORD type;
		DWORD id;
		DWORD locId;
		char serialNumber[16];
		char description[64];

		FT_STATUS ftStatus = FT_GetDeviceInfoDetail(i, &flags, &type, &id, &locId, serialNumber, description, NULL);
		if (ftStatus == FT_OK)
		{
			if (string(description).length() != 0)
			{
				cout << i << ") " << description << " (S/N: " << serialNumber << " | ID: 0x" << hex << uppercase << id << ")" << endl;
			}
			else
			{
				cout << i << ") -- unable to claim device --" << endl;
			}
		}
		else
		{
			cerr << "Failed to get device info for device " << i << endl;
		}
	}
	cout << endl;
	cout << "Use -jtag <num> to select a device" << endl;
	cout << endl;

	return;
}

FT_STATUS FTDIDevice::open(DWORD deviceIndex)
{
	_deviceIndex = deviceIndex;

	// Open FTDI device handle
	FT_STATUS ftStatus = FT_Open(deviceIndex, &_ftHandle);
	if (ftStatus != FT_OK)
	{
		cerr << "Cannot open the device number " << deviceIndex << endl;
		return ftStatus;
	}

	// Get chip driver version
	DWORD dwDriverVer;
	ftStatus = FT_GetDriverVersion(_ftHandle, &dwDriverVer);
	if (ftStatus == FT_OK)
	{
		unsigned long majorVer = (dwDriverVer >> 16) & 0xFF;
		unsigned long minorVer = (dwDriverVer >> 8) & 0xFF;
		unsigned long buildVer = dwDriverVer & 0xFF;

		cout << "Device driver version: " << hex << majorVer << "." << minorVer << "." << buildVer << endl;
	}
	else
	{
		cerr << "Cannot get driver version for device " << deviceIndex << endl;
		return ftStatus;
	}

	ftStatus = _initMPSSEMode(); // Initialize MPSSE mode on the FTDI chip and get ready for JTAG usage
	if (ftStatus != FT_OK)
	{
		cerr << "Could not initialize MPSSE mode on device " << deviceIndex << endl;
		return ftStatus;
	}

	ftStatus = _resetJTAGStateMachine();
	if (ftStatus != FT_OK)
	{
		cerr << "Could not reset JTAG state machine on device " << deviceIndex << endl;
		return ftStatus;
	}

	_initCore1(); // Initialize core 1 (this will run all the programs)
	_initCore2(); // Initialize core 2 (this will be idle)

	return ftStatus;
}

FT_STATUS FTDIDevice::_initMPSSEMode()
{
	/*
		============================
		Configure port for MPSSE use
		============================
	*/
	cout << "Configuring port... " << flush;

	// Reset the FTDI chip
	FT_STATUS ftStatus = FT_ResetDevice(_ftHandle);
	if (ftStatus != FT_OK)
	{
		cerr << "Failed to reset device " << _deviceIndex << endl;
		FT_Close(_ftHandle);
		return ftStatus;
	}

	DWORD inTransferSize = 16384;  // 16 KB
	DWORD outTransferSize = 16384; // 16 KB

	// Set USB input and output sizes
	ftStatus = FT_SetUSBParameters(_ftHandle, inTransferSize, outTransferSize);
	if (ftStatus != FT_OK)
	{
		cerr << "Failed to set USB params on device " << _deviceIndex << endl;
		FT_Close(_ftHandle);
		return ftStatus;
	}

	// Purge the RX and TX buffers
	ftStatus = FT_Purge(_ftHandle, FT_PURGE_RX | FT_PURGE_TX);
	if (ftStatus != FT_OK)
	{
		cerr << "Failed to purge buffers on device " << _deviceIndex << endl;
		FT_Close(_ftHandle);
		return ftStatus;
	}

	// Set the latency timer to 1 ms, just a random value to test really
	ftStatus = FT_SetLatencyTimer(_ftHandle, 1);
	if (ftStatus != FT_OK)
	{
		cerr << "Failed to set latency timer on device " << _deviceIndex << endl;
		FT_Close(_ftHandle);
		return ftStatus;
	}

	// Set the read and write timeouts to 10ms
	ftStatus = FT_SetTimeouts(_ftHandle, 10, 10);
	if (ftStatus != FT_OK)
	{
		cerr << "Failed to set timeouts on device " << _deviceIndex << endl;
		FT_Close(_ftHandle);
		return ftStatus;
	}

	// Enable MPSSE mode
	ftStatus = FT_SetBitMode(_ftHandle, 0x0, FT_BITMODE_RESET);
	if (ftStatus != FT_OK)
	{
		cerr << "Failed to set bit mode reset on device " << _deviceIndex << endl;
		FT_Close(_ftHandle);
		return ftStatus;
	}

	ftStatus = FT_SetBitMode(_ftHandle, 0x0, FT_BITMODE_MPSSE);
	if (ftStatus != FT_OK)
	{
		cerr << "Failed to set bit mode MPSSE on device " << _deviceIndex << endl;
		FT_Close(_ftHandle);
		return ftStatus;
	}

	sleep(1); // Wait for all the USB stuff to complete and work, 1s
	cout << "done!" << endl;
	cout << "Configuring MPSSE... " << flush;

	/*
		===============
		Configure MPSSE
		===============
		--> Synchronization & Bad Command Detection
	*/
	DWORD dwClockDivisor = 0x0004; // Value of clock divisor, SCL Frequency = 60/((1+0x0004)*2) (MHz) = 6 Mhz --> Faster values might introduce issues!
	BYTE byOutputBuffer[8];		   // Buffer to hold MPSSE commands and data to be sent to the FT2232H
	BYTE byInputBuffer[8];		   // Buffer to hold data read from the FT2232H
	DWORD dwCount = 0;			   // General loop index
	DWORD dwNumBytesToSend = 0;	   // Index to the output buffer
	DWORD dwNumBytesSent = 0;	   // Count of actual bytes sent - used with FT_Write
	DWORD dwNumBytesToRead = 0;	   // Number of bytes available to read in the driver's input buffer
	DWORD dwNumBytesRead = 0;	   // Count of actual bytes read - used with FT_Read

	// Enable internal loop-back
	byOutputBuffer[dwNumBytesToSend++] = 0x84;										   // Enable loopback
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the loopback command
	dwNumBytesToSend = 0;															   // Reset output buffer pointer
	ftStatus |= FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Check the receive buffer - it should be empty

	// Get the number of bytes in the FT2232H receive buffer
	if (dwNumBytesToRead != 0)
	{
		cerr << "Error - MPSSE receive buffer should be empty: " << ftStatus << endl;
		FT_SetBitMode(_ftHandle, 0x0, 0x00); // Reset the port to disable MPSSE
		FT_Close(_ftHandle);				 // Close the USB port
		return 1;							 // Exit with error
	}

	// -----------------------------------------------------------
	// Synchronize the MPSSE by sending a bogus opcode (0xAB),
	// The MPSSE will respond with "Bad Command" (0xFA) followed by
	// the bogus opcode itself.
	// -----------------------------------------------------------
	byOutputBuffer[dwNumBytesToSend++] = 0xAB;											// Add bogus command ‘0xAB’ to the queue
	ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the BAD command
	dwNumBytesToSend = 0;																// Reset output buffer pointer
	do
	{
		ftStatus |= FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead); // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));		 // or Timeout

	bool bCommandEchod = false;
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

	for (dwCount = 0; dwCount < dwNumBytesRead - 1; dwCount++)
	{
		if ((byInputBuffer[dwCount] == 0xFA) && (byInputBuffer[dwCount + 1] == 0xAB)) // Check if Bad command and echo command are received
		{
			bCommandEchod = true;
			break;
		}
	}

	if (bCommandEchod == false)
	{
		cerr << "Error in synchronizing the MPSSE" << endl;
		FT_Close(_ftHandle);
		return 1; // Exit with error
	}

	// Disable internal loop-back
	byOutputBuffer[dwNumBytesToSend++] = 0x85;											// Disable loopback
	ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the loopback command
	dwNumBytesToSend = 0;																// Reset output buffer pointer

	ftStatus |= FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead); // Check the receive buffer - it should be empty
	if (dwNumBytesToRead != 0)									 // Get the number of bytes in the FT2232H receive buffer
	{
		cerr << "Error - MPSSE receive buffer should be empty: " << ftStatus << endl;
		FT_SetBitMode(_ftHandle, 0x0, 0x00);
		// Reset the port to disable MPSSE
		FT_Close(_ftHandle); // Close the USB port
		return 1;			 // Exit with error
	}

	/*
		===============
		Configure MPSSE
		===============
		--> MPSSE Setup
	*/
	// -----------------------------------------------------------
	// Configure the MPSSE settings for JTAG
	// Multple commands can be sent to the MPSSE with one FT_Write
	// -----------------------------------------------------------
	dwNumBytesToSend = 0; // Start with a fresh index

	// Set up the Hi-Speed specific commands for the FTx232H
	byOutputBuffer[dwNumBytesToSend++] = 0x8A;											// Use 60MHz master clock (disable divide by 5)
	byOutputBuffer[dwNumBytesToSend++] = 0x97;											// Turn off adaptive clocking (may be needed for ARM)
	byOutputBuffer[dwNumBytesToSend++] = 0x8D;											// Disable three-phase clocking
	ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the HS-specific commands
	dwNumBytesToSend = 0;																// Reset output buffer pointer

	// Set TCK frequency
	// TCK = 60MHz /((1 + [(1 +0xValueH*256) OR 0xValueL])*2)
	byOutputBuffer[dwNumBytesToSend++] = '\x86';										// Command to set clock divisor
	byOutputBuffer[dwNumBytesToSend++] = dwClockDivisor & 0xFF;							// Set 0xValueL of clock divisor
	byOutputBuffer[dwNumBytesToSend++] = (dwClockDivisor >> 8) & 0xFF;					// Set 0xValueH of clock divisor
	ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the clock divisor commands
	dwNumBytesToSend = 0;																// Reset output buffer pointer

	// Set initial states of the MPSSE interface - low byte, both pin directions and output values
	// Pin name 	Signal 	Direction 	Config 	Initial State 	Config
	// ADBUS0 		TCK 	output 		1 		low 			0
	// ADBUS1 		TDI 	output 		1 		low 			0
	// ADBUS2 		TDO 	input 		0 						0
	// ADBUS3 		TMS 	output 		1 		high 			1
	// ADBUS4 		GPIOL0 	input 		0 						0
	// ADBUS5 		GPIOL1 	input 		0 						0
	// ADBUS6 		GPIOL2 	input 		0 						0
	// ADBUS7 		GPIOL3 	input 		0 						0
	byOutputBuffer[dwNumBytesToSend++] = 0x80;											// Configure data bits low-byte of MPSSE port
	byOutputBuffer[dwNumBytesToSend++] = 0b00001000;									// Initial state config above
	byOutputBuffer[dwNumBytesToSend++] = 0b00001011;									// Direction config above
	ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the low GPIO config commands
	dwNumBytesToSend = 0;																// Reset output buffer pointer

	// Note that since the data in subsequent sections will be clocked on the rising edge, the
	// inital clock state of high is selected. Clocks will be generated as high-low-high.
	//
	// For example, in this case, data changes on the rising edge to give it enough time
	// to have it available at the device, which will accept data *into* the target device
	// on the falling edge.

	// Set initial states of the MPSSE interface
	// - high byte, both pin directions and output values
	// Pin name 	Signal 	Direction 	Config 	Initial State 	Config
	// ACBUS0 		GPIOH0 	input 		0 						0
	// ACBUS1 		GPIOH1 	input 		0 						0
	// ACBUS2 		GPIOH2 	input 		0 						0
	// ACBUS3 		GPIOH3 	input 		0 						0
	// ACBUS4 		GPIOH4 	input 		0 						0
	// ACBUS5 		GPIOH5 	input 		0 						0
	// ACBUS6 		GPIOH6 	input 		0 						0
	// ACBUS7 		GPIOH7 	input 		0 						0
	// Configure data bits low-byte of MPSSE port
	byOutputBuffer[dwNumBytesToSend++] = 0x82;											// Configure data bits low-byte of MPSSE port
	byOutputBuffer[dwNumBytesToSend++] = 0x00;											// Initial state config above
	byOutputBuffer[dwNumBytesToSend++] = 0x00;											// Direction config above
	ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the high GPIO config commands
	dwNumBytesToSend = 0;																// Reset output buffer pointer

	for (dwCount = 0; dwCount < 8; dwCount++)
	{ // Clear out the input and output buffers
		byInputBuffer[dwCount] = 0x00;
		byOutputBuffer[dwCount] = 0x00;
	}

	// Check for any errors!
	if (ftStatus != FT_OK)
	{
		cerr << "Failed to config MPSSE on device " << _deviceIndex << endl;
		FT_Close(_ftHandle);
		return ftStatus;
	}

	cout << "done!" << endl;

	return ftStatus;
}

FT_STATUS FTDIDevice::_resetJTAGStateMachine()
{
	// Send TMS HIGH 5 times in a row to get back to Test-Logic-Reset
	BYTE byOutputBuffer[3];		// Buffer to hold MPSSE commands and data to be sent to the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write

	byOutputBuffer[dwNumBytesToSend++] = 0x4B;													 // Clock Data to TMS pin (no read), clock out negative edge
	byOutputBuffer[dwNumBytesToSend++] = 0x04;													 // Number of clock pulses = Length + 1 (5 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00111111;											 // Bit 7 is used to hold TDI/DO before the first clk of TMS
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Could not reset JTAG state machine on device " << _deviceIndex << endl;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	return ftStatus;
}

void FTDIDevice::_initCore1()
{
	/* make sure the memcfg regs are set properly
	 * XXX should be part of a board_init(), for now these are values which
	 * are guaranteed to work with the GR712RC eval board only!
	 *
	 * NOTE: this is done by probing the SRAM/SDRAM range via the BANK_SIZE
	 * setting in the MEMCFG regs; there you adjust the total size and write
	 * to one of the last few words in the bank, then shrink it by one power
	 * of 2 and repeat; when the minimum bank size is reached, do the
	 * reverse and read back the values. these should be unique, e.g.
	 * the actual address i.e. always store 0x4ffffffc to 0x4ffffffc
	 * and so on. If you read back the expected value, that is a verified
	 * upper limit of the bank; adjust up one power of 2 and repeat until
	 * you get nonsense back or the maximum bank size is reached.
	 * In other words:
	 *	1) BS = 0x200000, write 0x1fffc to last dword at 0x1fffc
	 *	2) reduce SRAM BANK SIZE config by 1
	 *	3) BS is now 0x10000, write 0xfffc to 0xfffc
	 *	4) repeat until bank size config is 0 => 8 kiB bank
	 *	5) now read back from 0x1ffc
	 *	6) if result == 0x1ffc, range is valid, increase bank size pwr by 1
	 *	7) now read back from 0x3ffc
	 *	8) if result == 0x3ffc, range is valid, increase bank size pwr by 1
	 *	   else end of physical ram was reached, decrement bank size by one
	 */
	this->iowrite32(0x80000000, 0x0003c0ff);
	this->iowrite32(0x80000004, 0x9a20546a);
	this->iowrite32(0x80000008, 0x0826e028);
	this->iowrite32(0x8000000c, 0x00000028);

	/* make sure all timers are stopped, in particular timer 4 (watchdog) */
	this->iowrite32(0x80000318, 0x0);
	this->iowrite32(0x80000328, 0x0);
	this->iowrite32(0x80000338, 0x0);
	this->iowrite32(0x80000348, 0x0);
}

void FTDIDevice::_initCore2()
{
	// Initialize the debug support unit for one CPU core
	// Taken from setup.c: sxi_dpu_setup_cpu_entry()
	cout << "Configuring CPU core 2 idle... " << flush;

	/* XXX this is needed for SXI DPU; the boot sw cannot start
	 * in SMP mode, so the second CPU is still at 0x0 initially
	 *
	 * we temporarily do this here to get ready for the EMC test
	 *
	 * we use the LEON3 debug support unit for this
	 */
	uint32_t tmp;

	dsu_set_noforce_debug_mode(1);
	dsu_set_cpu_break_on_iu_watchpoint(1);

	dsu_set_force_debug_on_watchpoint(1);

	// Set trap base register to be the same as on CPU0 and point %pc and %npc there
	tmp = dsu_get_reg_tbr(0) & ~0xfff;

	dsu_set_reg_tbr(1, tmp);
	dsu_set_reg_pc(1, tmp);
	dsu_set_reg_npc(1, tmp + 0x4);

	dsu_clear_iu_reg_file(1);
	// Default invalid mask
	dsu_set_reg_wim(1, 0x2);

	// Set CWP to 7
	dsu_set_reg_psr(1, 0xf34010e1);

	dsu_clear_cpu_break_on_iu_watchpoint(1);
	// Resume cpu 1
	dsu_clear_force_debug_on_watchpoint(1);

	dsu_clear_cpu_error_mode(1);

	cout << "done!" << endl;
}

void FTDIDevice::runCPU(BYTE cpuID)
{
	/*
	BYTE byOutputBuffer[10];	// Buffer to hold MPSSE commands and data to be sent to the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return;
	}

	// Goto Run-Test/Idle
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;													 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x00;													 // Number of clock pulses = Length + 1 (1 bit here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000000;											 // Data is shifted LSB first
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while querying ID for device " << _deviceIndex << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer
	*/

	// Stop the CPU core, set it to the beginning of the memory and wake it up again to execute the binary in memory
	cout << hex << "Halt Mode: " << dsu_get_cpu_in_halt_mode(cpuID) << " Error Mode: " << dsu_get_cpu_in_error_mode(cpuID) << endl;
	// cout << "CPU 1 Status: " << dsu_get_cpu_state(0) << endl; // 1 = power down, 0 = running
	// cout << "CPU 2 Status: " << dsu_get_cpu_state(1) << endl; // 1 = power down, 0 = running
	cout << hex << "Trap Reg: 0x" << dsu_get_reg_trap(cpuID) << endl;

	const uint32_t addr = 0x40000000;

	dsu_set_noforce_debug_mode(cpuID);
	dsu_set_cpu_break_on_iu_watchpoint(cpuID);
	dsu_set_cpu_halt_mode(cpuID);

	cout << hex << "Halt Mode: " << dsu_get_cpu_in_halt_mode(cpuID) << " Error Mode: " << dsu_get_cpu_in_error_mode(cpuID) << endl;
	// cout << "CPU 1 Status: " << dsu_get_cpu_state(0) << endl; // 1 = power down, 0 = running
	// cout << "CPU 2 Status: " << dsu_get_cpu_state(1) << endl; // 1 = power down, 0 = running
	cout << hex << "Trap Reg: 0x" << dsu_get_reg_trap(cpuID) << endl;

	dsu_set_force_debug_on_watchpoint(cpuID);

	dsu_set_reg_tbr(cpuID, addr);
	dsu_set_reg_pc(cpuID, addr);
	dsu_set_reg_npc(cpuID, addr + 0x4);

	dsu_clear_iu_reg_file(cpuID);

	dsu_set_reg_wim(cpuID, 0x2);		// Default invalid mask
	dsu_set_reg_psr(cpuID, 0xf34010e1); // Set CWP to 7

	const uint32_t start = 0x40000000 + 8 * 1024 * 1024; // Set to start of RAM + 8 MiB

	dsu_set_reg_sp(cpuID, 1, start);
	dsu_set_reg_fp(cpuID, 1, start);

	dsu_set_cpu_wake_up(cpuID); // CPU wake from setup.c
	// dsu_clear_cpu_break_on_iu_watchpoint(cpuID); // Not strictly needed with the iowrite down below
	dsu_clear_force_debug_on_watchpoint(cpuID); // Needed to resume cpu
	// dsu_clear_cpu_error_mode(cpuID); // Not strictly needed with the iowrite down below
	// dsu_clear_cpu_halt_mode(cpuID); // Not strictly needed with the iowrite down below

	this->iowrite32(0x90000000, 0x000002ef); // ACTUALLY RESUMES CPU

	cout << hex << "Halt Mode: " << dsu_get_cpu_in_halt_mode(cpuID) << " Error Mode: " << dsu_get_cpu_in_error_mode(cpuID) << endl;
	// cout << "CPU 1 Status: " << dsu_get_cpu_state(0) << endl; // 1 = power down, 0 = running
	// cout << "CPU 2 Status: " << dsu_get_cpu_state(1) << endl; // 1 = power down, 0 = running
	cout << hex << "Trap Reg: 0x" << dsu_get_reg_trap(cpuID) << endl;

	sleep(5); // Simple dummy execution time

	cout << hex << "Halt Mode: " << dsu_get_cpu_in_halt_mode(cpuID) << " Error Mode: " << dsu_get_cpu_in_error_mode(cpuID) << endl;
	// cout << "CPU 1 Status: " << dsu_get_cpu_state(0) << endl; // 1 = power down, 0 = running
	// cout << "CPU 2 Status: " << dsu_get_cpu_state(1) << endl; // 1 = power down, 0 = running
	cout << hex << "Trap Reg: 0x" << dsu_get_reg_trap(cpuID) << endl;

	cout << "done!" << endl;
}

BYTE FTDIDevice::getJTAGCount()
{
	BYTE byOutputBuffer[100];	// Buffer to hold MPSSE commands and data to be sent to the FT2232H
	BYTE byInputBuffer[100];	// Buffer to hold data read from the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write
	DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
	DWORD dwNumBytesRead = 0;	// Count of actual bytes read - used with FT_Read

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return 0;
	}

	// Goto Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x05;		 // Number of clock pulses = Length + 1 (6 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00001101; // Data is shifted LSB first, so actually 101100

	// Write one byte of 0xFF, but leave immediately to Exit-IR on this last byte with TMS.
	byOutputBuffer[dwNumBytesToSend++] = 0x1B; // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x06; // Length + 1 (7 bits here), only 7 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = 0xFF; // Ones only

	// Clock out last bit of the 0xFF byte and leave to Exit-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock bits out to TMS with read, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;		 // Length + 1 (1 bit here)
	byOutputBuffer[dwNumBytesToSend++] = 0b10000011; // One only

	// We are in Exit1-IR, go to Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;		 // Number of clock pulses = Length + 1 (4 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011; // Data is shifted LSB first

	// Send plenty of 0s into the DR registers to flush them
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x1B; // Clock data bits out with read
		byOutputBuffer[dwNumBytesToSend++] = 0x07; // Length + 1 (8 bits here)
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;													 // Clock bits in by reading
	byOutputBuffer[dwNumBytesToSend++] = 0x07;													 // Length + 1 (8 bits here)
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesRead); // Send off the commands
	dwNumBytesToSend = 0;																		 // Reset output buffer pointer

	if (ftStatus != FT_OK && dwNumBytesRead != 42)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return 0;
	}

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

	BYTE numberOfJTAGs = 0;

	// Now clock out 100 1s max with read until we receive one back
	for (BYTE i = 0; i < 100; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x3B;											// Clock bits out with read
		byOutputBuffer[dwNumBytesToSend++] = 0x00;											// Length + 1 (1 bit here)
		byOutputBuffer[dwNumBytesToSend++] = 0xFF;											// Ones only
		ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the commands
		dwNumBytesToSend = 0;																// Reset output buffer pointer

		if (ftStatus != FT_OK || dwNumBytesSent != 3)
		{
			cerr << "Error while scanning for number of JTAG devices with device " << _deviceIndex << endl;
			return 0;
		}

		do
		{
			ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
		} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
		ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

		if (ftStatus != FT_OK || dwNumBytesRead == 0)
		{
			cerr << "Error while reading number of JTAG devices with device " << _deviceIndex << endl;
			return 0;
		}

		if (byInputBuffer[0] != 0x00) // Received a 1 back
		{
			numberOfJTAGs = i;
			break;
		}
	}

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return 0;
	}

	return numberOfJTAGs;
}

DWORD FTDIDevice::readIDCODE()
{
	BYTE byOutputBuffer[10];	// Buffer to hold MPSSE commands and data to be sent to the FT2232H
	BYTE byInputBuffer[10];		// Buffer to hold data read from the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write
	DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
	DWORD dwNumBytesRead = 0;	// Count of actual bytes read - used with FT_Read

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return 0;
	}

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		 // Number of clock pulses = Length + 1 (5 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000101; // Data is shifted LSB first, so actually 10100

	// Clock out read command
	byOutputBuffer[dwNumBytesToSend++] = 0x28; // Read Bytes back
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // 3 + 1 Bytes = 32 bit IDCODE
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command
	dwNumBytesToSend = 0;																		 // Reset output buffer pointer

	if (ftStatus != FT_OK || dwNumBytesSent != 6)
	{
		cerr << "Error while querying ID for device " << _deviceIndex << endl;
		return 0;
	}

	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

	if (ftStatus != FT_OK)
	{
		cerr << "Error while reading ID for device " << _deviceIndex << endl;
		return 0;
	}

	if (dwNumBytesRead != 4)
	{
		cerr << "Device did not return the correct number of bytes for IDCODE!" << endl;
		return 0;
	}

	// Some formatting to get the right order of bytes for IDCODE
	DWORD id = (DWORD)((unsigned char)(byInputBuffer[3]) << 24 | (unsigned char)(byInputBuffer[2]) << 16 | (unsigned char)(byInputBuffer[1]) << 8 | (unsigned char)(byInputBuffer[0]));

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return 0;
	}

	return id;
}

BYTE FTDIDevice::scanIRLength()
{
	BYTE byOutputBuffer[10];	// Buffer to hold MPSSE commands and data to be sent to the FT2232H
	BYTE byInputBuffer[10];		// Buffer to hold data read from the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write
	DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
	DWORD dwNumBytesRead = 0;	// Count of actual bytes read - used with FT_Read

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR first
	{
		return 0;
	}

	// Goto Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x05;		 // Number of clock pulses = Length + 1 (6 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00001101; // Data is shifted LSB first, so the TMS pattern is 101100

	// Flush IR with a byte of zeros, more than enough for 6-bit IR
	byOutputBuffer[dwNumBytesToSend++] = 0x1B; // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x07; // Length + 1 (8 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only

	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;													 // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x07;													 // Length + 1 (8 bits here)
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return 0;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus |= FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);				   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

	BYTE lengthIR = 0;

	// Clock out 100 bits of 1s max with read back enabled, scan until a 1 gets back
	for (BYTE i = 0; i < 100; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x3B;											// Clock bits out with read
		byOutputBuffer[dwNumBytesToSend++] = 0x00;											// Length + 1 (1 bit here)
		byOutputBuffer[dwNumBytesToSend++] = 0xFF;											// Ones only
		ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

		if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend) // Check if everything has been sent!
		{
			cerr << "Error in IR length scan for device " << _deviceIndex << endl;
			return 0;
		}
		dwNumBytesToSend = 0; // Reset output buffer pointer

		do
		{
			ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
		} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
		ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

		if (ftStatus != FT_OK || dwNumBytesRead != dwNumBytesToRead)
		{
			cerr << "Error while reading length of IR with device " << _deviceIndex << endl;
			return 0;
		}

		if (byInputBuffer[0] != 0x00) // A 1 has finally come through, this is the limit, abort.
		{
			lengthIR = i;
			break;
		}
	}

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return 0;
	}

	return lengthIR; // Exit with success
}

void FTDIDevice::scanInstructionCodes(BYTE bitLengthIR)
{
	cout << "Scanning for IR opcodes that return a non-zero DR length. This might take a while..." << endl;

	BYTE maxIRLength = 0; // Get the highest opcode possible for the IR length

	for (BYTE i = 0; i < bitLengthIR; i++)
	{
		maxIRLength |= 1 << i;
	}

	unsigned int numberOfInstructions = 0;

	for (BYTE i = 0b00000000; i <= maxIRLength; i++)
	{
		unsigned int length = scanDRLength(i); // Grab individual DR Data register length
		if (length != 0)
		{
			cout << "- DR length for address 0x" << hex << uppercase << (unsigned int)i << ": " << dec << length << " bit" << endl;
			numberOfInstructions++;
		}
	}

	cout << "Scan complete! Found " << numberOfInstructions << " instructions." << endl;
}

BYTE FTDIDevice::scanDRLength(BYTE opcode)
{
	BYTE byOutputBuffer[100];	// Buffer to hold MPSSE commands and data to be sent to the FT2232H
	BYTE byInputBuffer[100];	// Buffer to hold data read from the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write
	DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
	DWORD dwNumBytesRead = 0;	// Count of actual bytes read - used with FT_Read

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return 0;
	}

	// Goto Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x05;		 // Number of clock pulses = Length + 1 (6 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00001101; // Data is shifted LSB

	// Clock out Command/Address register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;	 // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;	 // Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = opcode; // First 5 bits of the opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;				// Clock bits out to TMS, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;				// Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (opcode << 2) | 1; // Last bit of the opcode and leave Shift-IR

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;		 // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011; // Data is shifted LSB first, so the TMS pattern is 1100

	// TMS is currently low.
	// State machine is in Shift-DR, so now use the TDI/TDO command to shift 0's out TDI/DO while reading TDO/DI
	// Clock out 10 x 8 bits of 0s only to clear any other values
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;													 // Read back bits
	byOutputBuffer[dwNumBytesToSend++] = 0x07;													 // Length + 1 (8 bits here)
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return 0;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

	BYTE lengthDR = 0;

	// Clock out 100 bits of 1s with read back enabled until a 1 comes back out
	for (BYTE i = 0; i < 100; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x3B;											// Clock bits out with read
		byOutputBuffer[dwNumBytesToSend++] = 0x00;											// Length + 1 (1 bit here)
		byOutputBuffer[dwNumBytesToSend++] = 0xFF;											// Ones only
		ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

		if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
		{
			cerr << "Error in DR length scan for device " << _deviceIndex << " (opcode 0x" << hex << uppercase << CODE_DATA << ")" << endl;
			return 0;
		}
		dwNumBytesToSend = 0; // Reset output buffer pointer

		do
		{
			ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
		} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
		ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

		if (ftStatus != FT_OK || dwNumBytesRead != dwNumBytesToRead)
		{
			cerr << "Error while reading length of DR with device " << _deviceIndex << " (opcode 0x" << hex << uppercase << CODE_DATA << ")" << endl;
			return 0;
		}

		if (byInputBuffer[0] != 0x00) // Received a one back
		{
			lengthDR = i;
			break;
		}
	}

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return 0;
	}

	return lengthDR; // Exit with success
}

BYTE FTDIDevice::ioread8(DWORD addr)
{
	DWORD bigData = ioread32(addr);

	BYTE byte0 = (bigData & 0xFF);		   // First Byte of DWORD data
	BYTE byte1 = ((bigData >> 8) & 0xFF);  // Second Byte of DWORD data
	BYTE byte2 = ((bigData >> 16) & 0xFF); // Third Byte of DWORD data
	BYTE byte3 = ((bigData >> 24) & 0xFF); // Last Byte of DWORD data

	BYTE data;
	BYTE lsb = addr & 0xFF;

	if (lsb % 4 == 0) // First byte of a 4-byte DWORD data block
	{
		data = byte3;
	}
	else if ((lsb - 1) % 4 == 0) // Second byte of a 4-byte DWORD data block
	{
		data = byte2;
	}
	else if ((lsb - 2) % 4 == 0) // Third byte of a 4-byte DWORD data block
	{
		data = byte1;
	}
	else // Fourth byte of a 4-byte DWORD data block
	{
		data = byte0;
	}

	return data;
}

WORD FTDIDevice::ioread16(DWORD addr)
{
	DWORD bigData = ioread32(addr);

	WORD byte0 = (bigData & 0xFFFF);		 // First WORD of DWORD data
	WORD byte1 = ((bigData >> 16) & 0xFFFF); // Second WORD of DWORD data

	WORD data;
	BYTE lsb = addr & 0xFF;

	if (lsb % 4 == 0 || (lsb - 1) % 4 == 0) // First WORD of a 4-byte DWORD data block
	{
		data = byte1;
	}
	else // Second WORD of a 4-byte DWORD data block
	{
		data = byte0;
	}

	return data;
}

DWORD FTDIDevice::ioread32(DWORD addr)
{
	BYTE byOutputBuffer[100];	// Buffer to hold MPSSE commands and data to be sent to the FT2232H
	BYTE byInputBuffer[100];	// Buffer to hold data read from the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write
	DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
	DWORD dwNumBytesRead = 0;	// Count of actual bytes read - used with FT_Read

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return 0;
	}

	// Goto Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x05;		 // Number of clock pulses = Length + 1 (6 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00001101; // Data is shifted LSB first, so actually 101100

	// Clock out Command/Address register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;			 // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;			 // Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_ADDR_COMM; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;						// Clock bits out to TMS without read, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;						// Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_ADDR_COMM << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;		 // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011; // Data is shifted LSB first, so the TMS pattern is 1100

	// Clock out 10 x 8 bits of 0s only to clear out any other values
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;													 // Read back bits
	byOutputBuffer[dwNumBytesToSend++] = 0x07;													 // Length + 1 (8 bits here)
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return 0;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

	// Shift out AHB address DWORD (4 bytes)
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // Length + 1 (4 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;

	byOutputBuffer[dwNumBytesToSend++] = (addr & 0xFF);			// First Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 8) & 0xFF);	// Second Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 16) & 0xFF); // Third Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 24) & 0xFF); // Last Byte of DWORD address

	// Shift out 2-bit AHB transfer size: 10 for WORD
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;	   // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x01;	   // Length + 1 (2 bits here)
	byOutputBuffer[dwNumBytesToSend++] = RW_DWORD; // Read 32-bit DWORD

	// Shift out 1-bit Read/Write Instruction: 0x0 for READ while simultaneously leaving Shift-DR via TMS Exit-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x00;										   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000001;								   // Ones only
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while shifting out READ command for device " << _deviceIndex << endl;
		return 0;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Go to Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b0000111; // 11100

	// Clock out Data register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_DATA; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;				   // Clock bits out to TMS, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;				   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_DATA << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;										   // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011;								   // Data is shifted LSB first, so the TMS pattern is 1100
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return 0;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Clock out read command
	byOutputBuffer[dwNumBytesToSend++] = 0x28; // Read Bytes
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // 3 + 1 Bytes = 32 bit AHB Data -> Does not read SEQ Bit!
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while shifting out read instruction for device " << _deviceIndex << endl;
		return 0;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

	if (ftStatus != FT_OK)
	{
		cerr << "Error while reading data register for device " << _deviceIndex << endl;
		return 0;
	}

	if (dwNumBytesRead != dwNumBytesToRead)
	{
		cerr << "Bytes read: " << dec << dwNumBytesRead << endl;
		cerr << "Device did not return the correct number of bytes for the data register." << endl;
		return 0;
	}

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return 0;
	}

	// Format data
	return (DWORD)((unsigned char)(byInputBuffer[3]) << 24 | (unsigned char)(byInputBuffer[2]) << 16 | (unsigned char)(byInputBuffer[1]) << 8 | (unsigned char)(byInputBuffer[0]));
}

void FTDIDevice::ioread32raw(DWORD startAddr, DWORD *data, WORD size)
{
	if (size > 256) // Check 1kB boundary for SEQ transfers
	{
		cerr << "Warning: Size is bigger than recommended 1 kB maximum (GR712RC-UM)!" << endl;
	}

	BYTE byOutputBuffer[100];	// Buffer to hold MPSSE commands and data to be sent to the FT2232H
	BYTE byInputBuffer[100];	// Buffer to hold data read from the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write
	DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
	DWORD dwNumBytesRead = 0;	// Count of actual bytes read - used with FT_Read

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return;
	}

	// Goto Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x05;		 // Number of clock pulses = Length + 1 (6 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00001101; // Data is shifted LSB first, so actually 101100

	// Clock out Command/Address register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;			 // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;			 // Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_ADDR_COMM; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;						// Clock bits out to TMS without read, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;						// Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_ADDR_COMM << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;		 // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011; // Data is shifted LSB first, so the TMS pattern is 1100

	// Clock out 10 x 8 bits of 0s only to clear out any other values
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;													 // Read back bits
	byOutputBuffer[dwNumBytesToSend++] = 0x07;													 // Length + 1 (8 bits here)
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

	// Shift out AHB address DWORD (4 bytes)
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // Length + 1 (4 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;

	byOutputBuffer[dwNumBytesToSend++] = (startAddr & 0xFF);		 // First Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((startAddr >> 8) & 0xFF);	 // Second Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((startAddr >> 16) & 0xFF); // Third Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((startAddr >> 24) & 0xFF); // Last Byte of DWORD address

	// Shift out 2-bit AHB transfer size: 10 for WORD
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;	   // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x01;	   // Length + 1 (2 bits here)
	byOutputBuffer[dwNumBytesToSend++] = RW_DWORD; // Read 32-bit DWORD

	// Shift out 1-bit Read/Write Instruction: 0x0 for READ while simultaneously leaving Shift-DR via TMS Exit-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x00;										   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000001;								   // Ones only
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while shifting out READ command for device " << _deviceIndex << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Go to Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b0000111; // 11100

	// Clock out Data register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_DATA; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;				   // Clock bits out to TMS, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;				   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_DATA << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;										   // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011;								   // Data is shifted LSB first, so the TMS pattern is 1100
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	for (WORD i = 0; i < size; i++)
	{
		// Clock out read command
		byOutputBuffer[dwNumBytesToSend++] = 0x28; // Read Bytes
		byOutputBuffer[dwNumBytesToSend++] = 0x03; // 3 + 1 Bytes = 32 bit AHB Data -> Does not read SEQ Bit!
		byOutputBuffer[dwNumBytesToSend++] = 0x00;
		ftStatus |= FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

		if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
		{
			cerr << "Error while shifting out read instruction for device " << _deviceIndex << endl;
			break;
		}
		dwNumBytesToSend = 0; // Reset output buffer pointer

		do
		{
			ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
		} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
		ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer

		if (ftStatus != FT_OK)
		{
			cerr << "Error while reading data register for device " << _deviceIndex << endl;
			break;
		}

		if (dwNumBytesRead != dwNumBytesToRead)
		{
			cerr << "Bytes read: " << dec << dwNumBytesRead << endl;
			cerr << "Device did not return the correct number of bytes for the data register." << endl;
			break;
		}

		// Format data
		DWORD newData = (DWORD)((unsigned char)(byInputBuffer[3]) << 24 | (unsigned char)(byInputBuffer[2]) << 16 | (unsigned char)(byInputBuffer[1]) << 8 | (unsigned char)(byInputBuffer[0]));
		data[i] = newData;

		// Loop around once through Update-DR and then go back to Shift-DR
		byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock bits out without read
		byOutputBuffer[dwNumBytesToSend++] = 0x04;										   // Length + 1 (4 bits here)
		byOutputBuffer[dwNumBytesToSend++] = 0b10000111;								   // 11100
		ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

		if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
		{
			cerr << "Communication error with JTAG device!" << endl;
			break;
		}
		dwNumBytesToSend = 0; // Reset output buffer pointer
	}

	_resetJTAGStateMachine(); // Reset back to TLR
}

void FTDIDevice::ioread32(DWORD startAddr, DWORD *data, WORD size, bool progress)
{
	if (progress) // Optional terminal progress output
	{
		cout << "Reading data from memory... " << flush;
	}

	WORD readChunks = ceil((float)size / 256.0); // How many individual read chunks are needed

	for (WORD i = 0; i < readChunks; i++)
	{
		if (progress) // Optional terminal progress output
		{
			cout << "\rReading data from memory... " << dec << (unsigned int)((float)i / (float)(readChunks - 1.0) * 100.0) << "%  " << flush;
		}

		DWORD addr = startAddr + 1024 * i;
		WORD readSize; // Number of DWORDS to read

		if (i == readChunks - 1 && size % 256 != 0) // Last read is not a full 1024B chunk
		{
			readSize = size % 256; // Remainder
		}
		else
		{
			readSize = 256;
		}

		DWORD tempData[readSize];

		ioread32raw(addr, tempData, readSize);

		for (WORD j = 0; j < readSize; j++)
		{
			data[i * 256 + j] = tempData[j];
		}
	}

	if (progress) // Optional terminal progress output
	{
		cout << "\rReading data from memory... Complete!   " << endl;
	}
}

void FTDIDevice::iowrite8(DWORD addr, BYTE data)
{
	BYTE byOutputBuffer[100]; // Buffer to hold MPSSE commands and data to be sent to the FT2232H
	// BYTE byInputBuffer[100];	// Buffer to hold data read from the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write
	// DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
	// DWORD dwNumBytesRead = 0;	// Count of actual bytes read - used with FT_Read

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return;
	}

	// Goto Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x05;		 // Number of clock pulses = Length + 1 (6 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00001101; // Data is shifted LSB first, so actually 101100

	// Clock out Command/Address register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;			 // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;			 // Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_ADDR_COMM; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;						// Clock bits out to TMS without read, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;						// Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_ADDR_COMM << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;		 // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011; // Data is shifted LSB first, so the TMS pattern is 1100

	/*
	// Clock out 10 x 8 bits of 0s only to clear out any other values
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;													 // Read back bits
	byOutputBuffer[dwNumBytesToSend++] = 0x07;													 // Length + 1 (8 bits here)
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer
	*/

	// Shift out AHB address DWORD (4 bytes)
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // Length + 1 (4 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;

	byOutputBuffer[dwNumBytesToSend++] = (addr & 0xFF);			// First Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 8) & 0xFF);	// Second Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 16) & 0xFF); // Third Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 24) & 0xFF); // Last Byte of DWORD address

	// Shift out 2-bit AHB transfer size: 10 for WORD
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;	  // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x01;	  // Length + 1 (2 bits here)
	byOutputBuffer[dwNumBytesToSend++] = RW_BYTE; // Write 8-bit BYTE

	// Shift out 1-bit Read/Write Instruction: 0x1 for WRITE while simultaneously leaving Shift-DR via TMS Exit-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;													 // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x00;													 // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b10000001;											 // Ones only
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while shifting out WRITE command for device " << _deviceIndex << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Go to Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b0000111; // 11100

	// Clock out Data register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_DATA; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;				   // Clock bits out to TMS, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;				   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_DATA << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;										   // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011;								   // Data is shifted LSB first, so the TMS pattern is 1100
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	/*
	// Clock out 10 x 8 bits of 0s only to clear out any other values
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;										   // Read back bits
	byOutputBuffer[dwNumBytesToSend++] = 0x07;										   // Length + 1 (8 bits here)
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer
	*/

	// Shift out AHB data DWORD (1 byte)
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // Length + 1 (4 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;

	BYTE lsb = addr & 0xFF;

	if (lsb % 4 == 0) // First byte of a 4-byte DWORD data block
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // data; // Only byte of transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = data; // Dummy byte for transmission data
	}
	else if ((lsb - 1) % 4 == 0) // Second byte of a 4-byte DWORD data block
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // data; // Only byte of transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = data; // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Dummy byte for transmission data
	}
	else if ((lsb - 2) % 4 == 0) // Third byte of a 4-byte DWORD data block
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // data; // Only byte of transmission data
		byOutputBuffer[dwNumBytesToSend++] = data; // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Dummy byte for transmission data
	}
	else // Fourth byte of a 4-byte DWORD data block
	{
		byOutputBuffer[dwNumBytesToSend++] = data; // data; // Only byte of transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Dummy byte for transmission data
	}

	// Shift out 1-bit SEQ Transfer Instruction: 0x0 for single WRITE while simultaneously leaving Shift-DR via TMS Exit-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x00;										   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000001;								   // Only 1 to leave Shift-DR
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while shifting out data for device " << _deviceIndex << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return;
	}
}

void FTDIDevice::iowrite16(DWORD addr, WORD data)
{
	BYTE byOutputBuffer[100]; // Buffer to hold MPSSE commands and data to be sent to the FT2232H
	// BYTE byInputBuffer[100];	// Buffer to hold data read from the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write
	// DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
	// DWORD dwNumBytesRead = 0;	// Count of actual bytes read - used with FT_Read

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return;
	}

	// Goto Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x05;		 // Number of clock pulses = Length + 1 (6 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00001101; // Data is shifted LSB first, so actually 101100

	// Clock out Command/Address register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;			 // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;			 // Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_ADDR_COMM; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;						// Clock bits out to TMS without read, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;						// Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_ADDR_COMM << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;		 // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011; // Data is shifted LSB first, so the TMS pattern is 1100

	/*
	// Clock out 10 x 8 bits of 0s only to clear out any other values
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;													 // Read back bits
	byOutputBuffer[dwNumBytesToSend++] = 0x07;													 // Length + 1 (8 bits here)
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer
	*/

	// Shift out AHB address DWORD (4 bytes)
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // Length + 1 (4 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;

	byOutputBuffer[dwNumBytesToSend++] = (addr & 0xFF);			// First Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 8) & 0xFF);	// Second Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 16) & 0xFF); // Third Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 24) & 0xFF); // Last Byte of DWORD address

	// Shift out 2-bit AHB transfer size: 10 for WORD
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;	  // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x01;	  // Length + 1 (2 bits here)
	byOutputBuffer[dwNumBytesToSend++] = RW_WORD; // Write 16-bit WORD

	// Shift out 1-bit Read/Write Instruction: 0x1 for WRITE while simultaneously leaving Shift-DR via TMS Exit-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;													 // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x00;													 // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b10000001;											 // Ones only
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while shifting out WRITE command for device " << _deviceIndex << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Go to Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b0000111; // 11100

	// Clock out Data register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_DATA; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;				   // Clock bits out to TMS, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;				   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_DATA << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;										   // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011;								   // Data is shifted LSB first, so the TMS pattern is 1100
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	/*
	// Clock out 10 x 8 bits of 0s only to clear out any other values
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;										   // Read back bits
	byOutputBuffer[dwNumBytesToSend++] = 0x07;										   // Length + 1 (8 bits here)
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer
	*/

	// Shift out AHB data WORD (2 bytes)
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // Length + 1 (4 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;

	BYTE lsb = addr & 0xFF;

	if (lsb % 4 == 0 || (lsb - 1) % 4 == 0) // First byte of a 4-byte DWORD data block
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00;				   // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00;				   // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = (data & 0xFF);		   // First Byte of WORD data
		byOutputBuffer[dwNumBytesToSend++] = ((data >> 8) & 0xFF); // Second Byte of WORD data
	}
	else // Third byte of a 4-byte DWORD data block
	{
		byOutputBuffer[dwNumBytesToSend++] = (data & 0xFF);		   // First Byte of WORD data
		byOutputBuffer[dwNumBytesToSend++] = ((data >> 8) & 0xFF); // Second Byte of WORD data
		byOutputBuffer[dwNumBytesToSend++] = 0x00;				   // Dummy byte for transmission data
		byOutputBuffer[dwNumBytesToSend++] = 0x00;				   // Dummy byte for transmission data
	}

	// Shift out 1-bit SEQ Transfer Instruction: 0x0 for single WRITE while simultaneously leaving Shift-DR via TMS Exit-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x00;										   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000001;								   // Only 1 to leave Shift-DR
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while shifting out data for device " << _deviceIndex << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return;
	}
}

void FTDIDevice::iowrite32(DWORD addr, DWORD data)
{
	BYTE byOutputBuffer[100]; // Buffer to hold MPSSE commands and data to be sent to the FT2232H
	// BYTE byInputBuffer[100];	// Buffer to hold data read from the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write
	// DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
	// DWORD dwNumBytesRead = 0;	// Count of actual bytes read - used with FT_Read

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return;
	}

	// Goto Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x05;		 // Number of clock pulses = Length + 1 (6 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00001101; // Data is shifted LSB first, so actually 101100

	// Clock out Command/Address register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;			 // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;			 // Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_ADDR_COMM; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;						// Clock bits out to TMS without read, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;						// Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_ADDR_COMM << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;		 // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011; // Data is shifted LSB first, so the TMS pattern is 1100

	/*
	// Clock out 10 x 8 bits of 0s only to clear out any other values
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;													 // Read back bits
	byOutputBuffer[dwNumBytesToSend++] = 0x07;													 // Length + 1 (8 bits here)
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer
	*/

	// Shift out AHB address DWORD (4 bytes)
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // Length + 1 (4 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;

	byOutputBuffer[dwNumBytesToSend++] = (addr & 0xFF);			// First Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 8) & 0xFF);	// Second Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 16) & 0xFF); // Third Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((addr >> 24) & 0xFF); // Last Byte of DWORD address

	// Shift out 2-bit AHB transfer size: 10 for WORD
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;	   // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x01;	   // Length + 1 (2 bits here)
	byOutputBuffer[dwNumBytesToSend++] = RW_DWORD; // Write 32-bit DWORD

	// Shift out 1-bit Read/Write Instruction: 0x1 for WRITE while simultaneously leaving Shift-DR via TMS Exit-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;													 // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x00;													 // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b10000001;											 // Ones only
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while shifting out WRITE command for device " << _deviceIndex << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Go to Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b0000111; // 11100

	// Clock out Data register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_DATA; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;				   // Clock bits out to TMS, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;				   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_DATA << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;										   // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011;								   // Data is shifted LSB first, so the TMS pattern is 1100
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	/*
	// Clock out 10 x 8 bits of 0s only to clear out any other values
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;										   // Read back bits
	byOutputBuffer[dwNumBytesToSend++] = 0x07;										   // Length + 1 (8 bits here)
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer
	*/

	// Shift out AHB data DWORD (4 bytes)
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // Length + 1 (4 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;

	byOutputBuffer[dwNumBytesToSend++] = (data & 0xFF);			// First Byte of DWORD data
	byOutputBuffer[dwNumBytesToSend++] = ((data >> 8) & 0xFF);	// Second Byte of DWORD data
	byOutputBuffer[dwNumBytesToSend++] = ((data >> 16) & 0xFF); // Third Byte of DWORD data
	byOutputBuffer[dwNumBytesToSend++] = ((data >> 24) & 0xFF); // Last Byte of DWORD data

	// Shift out 1-bit SEQ Transfer Instruction: 0x0 for single WRITE while simultaneously leaving Shift-DR via TMS Exit-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x00;										   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000001;								   // Only 1 to leave Shift-DR
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while shifting out data for device " << _deviceIndex << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return;
	}
}

void FTDIDevice::iowrite32raw(DWORD startAddr, DWORD *data, WORD size)
{
	if (size > 256) // Check 1kB boundary for SEQ transfers
	{
		cerr << "Warning: Size is bigger than recommended 1 kB maximum (GR712RC-UM)!" << endl;
	}

	BYTE byOutputBuffer[100]; // Buffer to hold MPSSE commands and data to be sent to the FT2232H
	// BYTE byInputBuffer[100];	// Buffer to hold data read from the FT2232H
	DWORD dwNumBytesToSend = 0; // Index to the output buffer
	DWORD dwNumBytesSent = 0;	// Count of actual bytes sent - used with FT_Write
	// DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
	// DWORD dwNumBytesRead = 0;	// Count of actual bytes read - used with FT_Read

	if (_resetJTAGStateMachine() != FT_OK) // Reset back to TLR
	{
		return;
	}

	// Goto Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x05;		 // Number of clock pulses = Length + 1 (6 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00001101; // Data is shifted LSB first, so actually 101100

	// Clock out Command/Address register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;			 // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;			 // Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_ADDR_COMM; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;						// Clock bits out to TMS without read, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;						// Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_ADDR_COMM << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;		 // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011; // Data is shifted LSB first, so the TMS pattern is 1100

	/*
	// Clock out 10 x 8 bits of 0s only to clear out any other values
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	for (BYTE i = 0; i < 10; i++)
	{
		byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
	}
	// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
	byOutputBuffer[dwNumBytesToSend++] = 0x2A;													 // Read back bits
	byOutputBuffer[dwNumBytesToSend++] = 0x07;													 // Length + 1 (8 bits here)
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Do a read to flush the read buffer, the data is not needed...
	do
	{
		ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
	ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer
	*/

	// Shift out AHB address DWORD (4 bytes)
	byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03; // Length + 1 (4 bytes here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;

	byOutputBuffer[dwNumBytesToSend++] = (startAddr & 0xFF);		 // First Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((startAddr >> 8) & 0xFF);	 // Second Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((startAddr >> 16) & 0xFF); // Third Byte of DWORD address
	byOutputBuffer[dwNumBytesToSend++] = ((startAddr >> 24) & 0xFF); // Last Byte of DWORD address

	// Shift out 2-bit AHB transfer size: 10 for WORD
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;	   // Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x01;	   // Length + 1 (2 bits here)
	byOutputBuffer[dwNumBytesToSend++] = RW_DWORD; // Write 32-bit DWORD

	// Shift out 1-bit Read/Write Instruction: 0x1 for WRITE while simultaneously leaving Shift-DR via TMS Exit-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;													 // Clock bits out with read
	byOutputBuffer[dwNumBytesToSend++] = 0x00;													 // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b10000001;											 // Ones only
	FT_STATUS ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Error while shifting out WRITE command for device " << _deviceIndex << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Go to Shift-IR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here)
	byOutputBuffer[dwNumBytesToSend++] = 0b0000111; // 11100

	// Clock out Data register opcode
	byOutputBuffer[dwNumBytesToSend++] = 0x1B;		// Clock bits out without read
	byOutputBuffer[dwNumBytesToSend++] = 0x04;		// Length + 1 (5 bits here), only 5 because the last one will be clocked by the next TMS command
	byOutputBuffer[dwNumBytesToSend++] = CODE_DATA; // First 5 bits of the 6-bit-long opcode

	// Clock out last bit of the opcode and leave to Exit-IR immediately
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;				   // Clock bits out to TMS, move out of Shift-IR and clock out the last TDI/DO bit
	byOutputBuffer[dwNumBytesToSend++] = 0x00;				   // Length + 1 (1 bits here)
	byOutputBuffer[dwNumBytesToSend++] = (CODE_DATA << 2) | 1; // Move the MSB of the opcode to the MSB of the BYTE, then make the first bit a 1 for TMS

	// Goto Shift-DR
	byOutputBuffer[dwNumBytesToSend++] = 0x4B;										   // Clock out TMS without read
	byOutputBuffer[dwNumBytesToSend++] = 0x03;										   // Number of clock pulses = Length + 1 (4 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0b00000011;								   // Data is shifted LSB first, so the TMS pattern is 1100
	ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

	if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
	{
		cerr << "Communication error with JTAG device!" << endl;
		return;
	}
	dwNumBytesToSend = 0; // Reset output buffer pointer

	// Iterate over data package and write individual DWORDs to memory
	for (WORD i = 0; i < size; i++)
	{
		/*
		// Clock out 10 x 8 bits of 0s only to clear out any other values
		byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
		byOutputBuffer[dwNumBytesToSend++] = 0x09; // Length + 1 (10 bytes here)
		byOutputBuffer[dwNumBytesToSend++] = 0x00;
		for (BYTE i = 0; i < 10; i++)
		{
			byOutputBuffer[dwNumBytesToSend++] = 0x00; // Zeros only
		}
		// Clock out Read: FIXES SOME ISSUES; I DONT KNOW WHY?!
		byOutputBuffer[dwNumBytesToSend++] = 0x2A;													 // Read back bits
		byOutputBuffer[dwNumBytesToSend++] = 0x07;													 // Length + 1 (8 bits here)
		ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

		if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
		{
			cerr << "Communication error with JTAG device!" << endl;
			return;
		}
		dwNumBytesToSend = 0; // Reset output buffer pointer

		// Do a read to flush the read buffer, the data is not needed...
		do
		{
			ftStatus = FT_GetQueueStatus(_ftHandle, &dwNumBytesToRead);					   // Get the number of bytes in the device input buffer
		} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));						   // or Timeout
		ftStatus |= FT_Read(_ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead); // Read out the data from input buffer
		*/

		DWORD dataWord = data[i];

		// Shift out AHB data DWORD (4 bytes)
		byOutputBuffer[dwNumBytesToSend++] = 0x19; // Clock bytes out without read
		byOutputBuffer[dwNumBytesToSend++] = 0x03; // Length + 1 (4 bytes here)
		byOutputBuffer[dwNumBytesToSend++] = 0x00;

		byOutputBuffer[dwNumBytesToSend++] = (dataWord & 0xFF);			// First Byte of DWORD data
		byOutputBuffer[dwNumBytesToSend++] = ((dataWord >> 8) & 0xFF);	// Second Byte of DWORD data
		byOutputBuffer[dwNumBytesToSend++] = ((dataWord >> 16) & 0xFF); // Third Byte of DWORD data
		byOutputBuffer[dwNumBytesToSend++] = ((dataWord >> 24) & 0xFF); // Last Byte of DWORD data

		// Shift out 1-bit SEQ Transfer Instruction: 0x1 for sequential WRITE while simultaneously leaving Shift-DR via TMS Exit-DR
		byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock bits out with read
		byOutputBuffer[dwNumBytesToSend++] = 0x00;		 // Length + 1 (1 bits here)
		byOutputBuffer[dwNumBytesToSend++] = 0b10000001; // Only 1 to leave Shift-DR with SEQ Bit

		if (i < size - 1) // Fixes an issue with subsequent _resetJTAGStateMachine clocking out another data point
		{
			// Loop around once through Update-DR and then go back to Shift-DR
			byOutputBuffer[dwNumBytesToSend++] = 0x4B;		 // Clock bits out without read
			byOutputBuffer[dwNumBytesToSend++] = 0x03;		 // Length + 1 (3 bits here)
			byOutputBuffer[dwNumBytesToSend++] = 0b00000011; // 1100
		}
		ftStatus = FT_Write(_ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent); // Send off the TMS command

		if (ftStatus != FT_OK || dwNumBytesSent != dwNumBytesToSend)
		{
			cerr << "Communication error with JTAG device!" << endl;
			break;
		}
		dwNumBytesToSend = 0; // Reset output buffer pointer
	}

	_resetJTAGStateMachine(); // Reset back to TLR
}

void FTDIDevice::iowrite32(DWORD startAddr, DWORD *data, WORD size, bool progress)
{
	if (progress) // Optional terminal progress output
	{
		cout << "Writing data to memory... " << flush;
	}

	WORD writeChunks = ceil((float)size / 256.0); // How many individual write chunks are needed

	for (WORD i = 0; i < writeChunks; i++)
	{
		if (progress) // Optional terminal progress output
		{
			cout << "\rWriting data to memory... " << dec << (unsigned int)((float)i / (float)(writeChunks - 1.0) * 100.0) << "%  " << flush;
		}

		DWORD addr = startAddr + 1024 * i;
		WORD writeSize; // Number of DWORDS to write

		if (i == writeChunks - 1 && size % 256 != 0) // Last write is not a full 1024B chunk
		{
			writeSize = size % 256; // Remainder
		}
		else
		{
			writeSize = 256;
		}

		DWORD tempData[writeSize];

		for (WORD j = 0; j < writeSize; j++)
		{
			tempData[j] = data[i * 256 + j];
		}

		iowrite32raw(addr, tempData, writeSize);
	}

	if (progress) // Optional terminal progress output
	{
		cout << "\rWriting data to memory... Complete!   " << endl;
	}
}
