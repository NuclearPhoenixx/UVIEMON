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

	const unsigned int writeSize = ceil(float(bytesRead) * sizeof(BYTE) / sizeof(DWORD));
	DWORD writeBuffer[writeSize];

	for (DWORD i = cutoffSize / 4 /* Divide by 4 to convert BYTES to DWORDS */; i < writeSize; i++) // Start after cutoff
	{
		writeBuffer[i] = (buffer[i * 4] << 24) | (buffer[i * 4 + 1] << 16) | (buffer[i * 4 + 2] << 8) | buffer[i * 4 + 3];
	}

	const DWORD addr = 0x40000000; // Begin writing from this address
	handle.iowrite32(addr, writeBuffer, writeSize, true);

	cout << "Loading file complete!" << endl;
}

void verify(FTDIDevice &handle, std::string &path)
{
	ifstream file;
	file.open(path, ios::binary | ios::ate);

	const streamsize size = file.tellg();

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

	const WORD writeSize = ceil(float(bytesRead) * sizeof(BYTE) / sizeof(DWORD));
	DWORD writeBuffer[writeSize];

	for (WORD i = 0; i < writeSize; i++)
	{
		writeBuffer[i] = (buffer[i * 4] << 24) | (buffer[i * 4 + 1] << 16) | (buffer[i * 4 + 2] << 8) | buffer[i * 4 + 3];
	}

	DWORD readBuffer[writeSize];

	const DWORD addr = 0x40000000; // Begin writing from this address
	handle.ioread32(addr, readBuffer, writeSize, true);

	cout << "Verifying file... " << flush;

	for (WORD i = 0; i < writeSize; i++)
	{
		if (readBuffer[i] != writeBuffer[i])
		{
			cerr << "\rVerifying file... ERROR! Byte " << dec << i << " incorrect." << endl;
			return;
		}
		cout << "\rVerifying file... " << dec << (unsigned int)((float)i / (float)(writeSize - 1) * 100.0) << "%    " << flush;
	}

	cout << "\rVerifying file... OK!    " << endl;
}
