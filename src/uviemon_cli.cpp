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

	cout << "Uploading File '" << path << "'..." << endl;
	cout << "File Size: " << dec << size << " B" << endl;
	cout << "Size Read: " << dec << bytesRead << " B" << endl;
	// TODO: Check for compatability or something?
	cout << endl;

	// Save cut-off header to an extra file to restore the full binary when downloading again
	ofstream wfile;
	wfile.open(path + "-header.tmp", ios::out | ios::binary);
	wfile.write(buffer, cutoffSize);
	wfile.close();

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
	cout << "Running executable..." << endl;

	// TODO: Check if something has been uploaded

	handle.runCPU(0); // Execute on CPU Core 1

	// TODO: Check if successfull, check for errors or OK exits
	// TODO: UART output
}

void reset(FTDIDevice &handle)
{
	cout << "Resetting... ";
	handle.reset();
	cout << "Done." << endl;
}