/*
	============================================
	uviemon: free(TM) replacement for grmon

	Functions used in the command line interface
	of uviemon. All these functions corrrespond
	to their command in the console to be called
	by the user.
	============================================
*/

#include "uviemon_cli.hpp"

#include <iostream> // cout & cerr
#include <iomanip>	// Used for setfill and setw (cout formatting)
#include <fstream>	// For loading a file in load()
#include <cmath>	// For std::ceil in load()

using namespace std;

std::unordered_map<int, std::string> tt_errors = {
	{0x0, "[reset]: Power-on reset"},
	{0x2b, "[write_error]: write buffer error"},
	{0x01, "[instruction_access_error]: Error during instruction fetch"},
	{0x02, "[illegal_instruction]: UNIMP or other un-implemented instruction"},
	{0x03, "[privileged_instruction]: Execution of privileged instruction in user mode"},
	{0x04, "[fp_disabled]: FP instruction while FPU disabled"},
	{0x24, "[cp_disabled]: CP instruction while Co-processor disabled. The GR712RC does not implement a co-processor and CP instructions will trigger this trap "},
	{0x0B, "[watchpoint_detected]: Hardware breakpoint match"},
	{0x05, "[window_overflow]: SAVE into invalid window"},
	{0x06, "[window_underflow]: RESTORE into invalid window"},
	{0x20, "[register_hadrware_error]: Uncorrectable register file EDAC error"},
	{0x07, "[mem_address_not_aligned]: Memory access to un-aligned address"},
	{0x08, "[fp_exception]: FPU exception"},
	{0x09, "[data_access_exception]: Access error during load or store instruction"},
	{0x0A, "[tag_overflow]: Tagged arithmetic overflow"},
	{0x2A, "[divide_exception]: Divide by zero"},
	{0x11, "[interrupt_level_1]: Asynchronous interrupt 1"},
	{0x12, "[interrupt_level_2]: Asynchronous interrupt 2"},
	{0x13, "[interrupt_level_3]: Asynchronous interrupt 3"},
	{0x14, "[interrupt_level_4]: Asynchronous interrupt 4"},
	{0x15, "[interrupt_level_5]: Asynchronous interrupt 5"},
	{0x16, "[interrupt_level_6]: Asynchronous interrupt 6"},
	{0x17, "[interrupt_level_7]: Asynchronous interrupt 7"},
	{0x18, "[interrupt_level_8]: Asynchronous interrupt 8"},
	{0x19, "[interrupt_level_9]: Asynchronous interrupt 9"},
	{0x1A, "[interrupt_level_10]: Asynchronous interrupt 10"},
	{0x1B, "[interrupt_level_11]: Asynchronous interrupt 11"},
	{0x1C, "[interrupt_level_12]: Asynchronous interrupt 12"},
	{0x1D, "[interrupt_level_13]: Asynchronous interrupt 13"},
	{0x1E, "[interrupt_level_14]: Asynchronous interrupt 14"},
	{0x1F, "[interrupt_level_15]: Asynchronous interrupt 15"},
	{0x80, "[trap_instruction]: OK"}
	// Anything larger than 0x80 will be some other Software trap instruction (TA)
};

string _hexToString(DWORD *data, size_t size)
{
	BYTE byteArray[size * sizeof(data[0])];

	for (size_t i = 0; i < size; i++)
	{
		byteArray[i * 4] = (BYTE)((data[i] >> 24) & 0xFF);
		byteArray[i * 4 + 1] = (BYTE)((data[i] >> 16) & 0xFF);
		byteArray[i * 4 + 2] = (BYTE)((data[i] >> 8) & 0xFF);
		byteArray[i * 4 + 3] = (BYTE)(data[i] & 0xFF);
	}

	string s = "";

	for (size_t i = 0; i < sizeof(byteArray); i++)
	{
		char newChar = char(byteArray[i]);

		if (newChar >= 32 && newChar <= 126)
		{
			s += newChar;
		}
		else
		{
			s += ".";
		}
	}

	return s;
}

string _hexToString(WORD *data, size_t size)
{
	BYTE byteArray[size * sizeof(data[0])];

	for (size_t i = 0; i < size; i++)
	{
		byteArray[i * 2] = (BYTE)((data[i] >> 8) & 0xFF);
		byteArray[i * 2 + 1] = (BYTE)(data[i] & 0xFF);
	}

	string s = "";

	for (size_t i = 0; i < sizeof(byteArray); i++)
	{
		char newChar = char(byteArray[i]);

		if (newChar >= 32 && newChar <= 126)
		{
			s += newChar;
		}
		else
		{
			s += ".";
		}
	}

	return s;
}

string _hexToString(BYTE *data, size_t size)
{
	string s = "";

	for (size_t i = 0; i < size; i++)
	{
		char newChar = char(data[i]);

		if (newChar >= 32 && newChar <= 126)
		{
			s += newChar;
		}
		else
		{
			s += ".";
		}
	}

	return s;
}

void wmem(FTDIDevice &handle, DWORD addr, DWORD data)
{
	cout << "Writing to memory... " << flush;
	handle.iowrite32(addr, data);
	cout << "OK!" << endl;
}

void wmemh(FTDIDevice &handle, DWORD addr, WORD data)
{
	cout << "Writing to memory... " << flush;
	handle.iowrite16(addr, data);
	cout << "OK!" << endl;
}

void wmemb(FTDIDevice &handle, DWORD addr, BYTE data)
{
	cout << "Writing to memory... " << flush;
	handle.iowrite8(addr, data);
	cout << "OK!" << endl;
}

void mem(FTDIDevice &handle, DWORD startAddr, DWORD length)
{
	BYTE showWidth = 4;
	DWORD arr[length];

	DWORD chars[showWidth];
	WORD arrayIndex = 0;

	handle.ioread32(startAddr, arr, length, length > 256); // Use sequential reads

	for (DWORD i = 0; i < length; i++)
	{
		if (i % showWidth == 0)
		{
			if (i > 0)
			{
				cout << _hexToString(chars, arrayIndex);

				arrayIndex = 0;

				cout << endl;
			}

			cout << hex << nouppercase << "0x" << setfill('0') << setw(8) << startAddr << "  " << flush;
			startAddr += 16;
		}

		cout << setfill('0') << setw(8) << hex << nouppercase << arr[i] << "  " << flush;

		chars[arrayIndex] = arr[i];
		arrayIndex++;
	}

	if (arrayIndex < showWidth)
	{
		cout << _hexToString(chars, arrayIndex) << endl;
	}
	else
	{
		cout << _hexToString(chars, showWidth) << endl;
	}
}

void memh(FTDIDevice &handle, DWORD startAddr, DWORD length)
{
	const DWORD maxAddr = startAddr + 2 * length;
	const BYTE arrSize = 8;

	WORD index = 0;
	WORD arrayIndex = 0;
	WORD arr[arrSize];

	// Loop through all the addresses and read out individual DWORDs
	for (DWORD addr = startAddr; addr < maxAddr; addr += 2)
	{
		if (index % arrSize == 0)
		{
			if (index > 0)
			{
				cout << _hexToString(arr, arrSize);
				arrayIndex = 0;

				cout << endl;
			}

			cout << hex << nouppercase << "0x" << setfill('0') << setw(8) << addr << "  " << flush;
		}

		WORD data = handle.ioread16(addr);
		arr[arrayIndex] = data;
		arrayIndex++;

		cout << setfill('0') << setw(4) << hex << nouppercase << data << "  " << flush;

		index++;
	}

	if (index < arrSize)
	{
		cout << _hexToString(arr, arrayIndex) << endl;
	}
	else
	{
		cout << _hexToString(arr, arrSize) << endl;
	}
}

void memb(FTDIDevice &handle, DWORD startAddr, DWORD length)
{
	const DWORD maxAddr = startAddr + 1 * length;
	const BYTE arrSize = 16;

	WORD index = 0;
	WORD arrayIndex = 0;
	BYTE arr[arrSize];

	// Loop through all the addresses and read out individual DWORDs
	for (DWORD addr = startAddr; addr < maxAddr; addr++)
	{
		if (index % arrSize == 0)
		{
			if (index > 0)
			{
				cout << _hexToString(arr, arrSize);
				arrayIndex = 0;

				cout << endl;
			}

			cout << hex << nouppercase << "0x" << setfill('0') << setw(8) << addr << "  " << flush;
		}

		WORD data = handle.ioread8(addr);
		arr[arrayIndex] = data;
		arrayIndex++;

		cout << setfill('0') << setw(2) << hex << nouppercase << data << "  " << flush;

		index++;
	}

	if (index < arrSize)
	{
		cout << _hexToString(arr, arrayIndex) << endl;
	}
	else
	{
		cout << _hexToString(arr, arrSize) << endl;
	}
}

void bdump(FTDIDevice &handle, DWORD startAddr, DWORD length, string &path)
{
	DWORD readBuffer[length];
	const DWORD charArraySize = length * sizeof(readBuffer[0]);
	char charArray[charArraySize];

	DWORD dwordLength = ceil(length / sizeof(DWORD)); // Convert byte length to DWORD length

	handle.ioread32(startAddr, readBuffer, dwordLength, true);

	for (DWORD i = 0; i < dwordLength; i++)
	{
		DWORD temp = readBuffer[i];
		charArray[i * 4] = (char)(temp >> 24);
		charArray[i * 4 + 1] = ((char)(temp >> 16) & 0xFF);
		charArray[i * 4 + 2] = ((char)(temp >> 8) & 0xFF);
		charArray[i * 4 + 3] = ((char)(temp)&0xFF);
	}

	ofstream file;

	file.open(path, ios::out | ios::binary);
	file.write(charArray, charArraySize);
	file.close();
}

void wash(FTDIDevice &handle, WORD size, DWORD addr, DWORD c)
{
	DWORD data[size] = {};

	for (WORD i = 0; i < size; i++)
	{
		data[i] = c;
	}

	cout << "Writing 0x" << hex << (unsigned int)c << " to " << dec << (unsigned int)size << " DWORD(s) in memory, starting at 0x" << hex << addr << "..." << endl;

	handle.iowrite32(addr, data, size, true);

	cout << "Wash of " << (unsigned int)size << " DWORD(s) complete!" << endl;
}

void load(FTDIDevice &handle, string &path)
{
	ifstream file;
	file.open(path, ios::binary | ios::ate);

	const streamsize size = file.tellg();
	const uint32_t cutoffSize = 64 * 1024; // Cut off the first 64 kiB of data (ELF-header + alignment section)

	if (size == -1)
	{
		cerr << "File not found!" << endl;
		return;
	}
	else if (size == 0)
	{
		cerr << "File is empty!" << endl;
		return;
	}
	else if (size < cutoffSize)
	{
		cerr << "File size is too small! Needs to be at least 64 kiB..." << endl;
		return;
	}

	char buffer[size] = {}; // Create buffer for individual BYTES of binary data

	file.seekg(0, ios::beg);

	if (!file.read(buffer, size))
	{
		cerr << "Error loading file '" << path << "'..." << endl;
		return;
	}

	const streamsize bytesRead = file.gcount(); // Number of bytes actually read
	file.close();

	cout << "Uploading File '" << path << "'..." << endl;
	cout << "File Size: " << dec << size << " B" << endl;
	cout << "Size Read: " << dec << bytesRead << " B" << endl;
	// TODO: Check for compatability or something?
	cout << endl;

	/*
	// Save cut-off header to an extra file to restore the full binary when downloading again
	ofstream wfile;
	wfile.open(path + "-header.tmp", ios::out | ios::binary);
	wfile.write(buffer, cutoffSize);
	wfile.close();
	*/

	const unsigned int offset = cutoffSize * sizeof(BYTE) / sizeof(DWORD);						   // Divide to convert BYTES to DWORDS
	const unsigned int writeSize = ceil(float(bytesRead) * sizeof(BYTE) / sizeof(DWORD)) - offset; // Divide to convert BYTES to DWORDS and subtract the offset
	DWORD writeBuffer[writeSize];

	for (DWORD i = offset; i < writeSize + offset; i++) // Start after cutoff
	{
		const DWORD wdword = ((unsigned char)(buffer[i * 4]) << 24) | ((unsigned char)(buffer[i * 4 + 1] & 0xFF) << 16) | ((unsigned char)(buffer[i * 4 + 2] & 0xFF) << 8) | ((unsigned char)(buffer[i * 4 + 3] & 0xFF));
		writeBuffer[i - offset] = wdword;
	}

	const DWORD addr = 0x40000000; // Begin writing to this address
	handle.iowrite32(addr, writeBuffer, writeSize, true);

	cout << "Loading file complete!" << endl;
}

void verify(FTDIDevice &handle, std::string &path)
{
	ifstream file;
	file.open(path, ios::binary | ios::ate);

	const streamsize size = file.tellg();
	const uint32_t cutoffSize = 64 * 1024; // Cut off the first 64 kiB of data (ELF-header + alignment section)

	if (size < cutoffSize)
	{
		cerr << "File size is too small! Needs to be at least 64 kiB..." << endl;
		return;
	}

	char buffer[size] = {}; // Create buffer for individual BYTES of binary data

	file.seekg(0, ios::beg);

	if (!file.read(buffer, size))
	{
		cerr << "Error loading file '" << path << "'..." << endl;
		return;
	}

	const streamsize bytesRead = file.gcount(); // Number of bytes actually read
	file.close();

	cout << "Verifying File '" << path << "'..." << endl;
	cout << "File Size: " << dec << size << " B" << endl;
	cout << "Size Read: " << dec << bytesRead << " B" << endl;
	// TODO: Check for compatability or something?
	cout << endl;

	const unsigned int offset = cutoffSize * sizeof(BYTE) / sizeof(DWORD);						  // Divide to convert BYTES to DWORDS
	const unsigned int readSize = ceil(float(bytesRead) * sizeof(BYTE) / sizeof(DWORD)) - offset; // Divide to convert BYTES to DWORDS and subtract the offset
	DWORD readBuffer[readSize];

	const DWORD addr = 0x40000000; // Begin writing from this address
	handle.ioread32(addr, readBuffer, readSize, true);

	cout << "Verifying file... " << flush;

	for (DWORD i = offset; i < readSize; i++) // Start after cutoff
	{
		const DWORD wdword = ((unsigned char)(buffer[i * 4]) << 24) | ((unsigned char)(buffer[i * 4 + 1] & 0xFF) << 16) | ((unsigned char)(buffer[i * 4 + 2] & 0xFF) << 8) | ((unsigned char)(buffer[i * 4 + 3] & 0xFF));

		if (readBuffer[i - offset] != wdword)
		{
			cerr << "\rVerifying file... ERROR! Byte " << dec << i << " incorrect." << endl;
			return;
		}
		cout << "\rVerifying file... " << dec << (unsigned int)((float)i / (float)(readSize - 1) * 100.0) << "%    " << flush;
	}

	cout << "\rVerifying file... OK!    " << endl;
}

void run(FTDIDevice &handle)
{
	cout << "Running executable...";

	// TODO: Check if something has been uploaded

	BYTE tt = handle.runCPU(0); // Execute on CPU Core 1

	cout << " Done." << endl;
	cout << endl;
	cout << "tt 0x" << hex << (unsigned int)tt << ", ";

	if (tt <= 0x80)
	{
		cout << tt_errors[tt] << endl;
	}
	else
	{
		cout << "[trap_instruction]: Software trap instruction (TA)" << endl;
	}

	// TODO: Check if successfull, check for errors or OK exits
	// TODO: UART output
}

void reset(FTDIDevice &handle)
{
	cout << "Resetting...";
	handle.reset();
	cout << " Done." << endl;
}