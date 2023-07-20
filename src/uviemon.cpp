/*
	========================================
	uviemon: free(TM) replacement for grmon

	This is the main routine running when
	uviemon gets executed. It consists of
	the command line params and the console.
	========================================
*/

#include "uviemon.hpp"

#include <iostream>			   // cout and cerr
#include <cstring>			   // Needed for strcmp
#include <string>			   // string for user input/output
#include <sstream>			   // Used in console user input parsing
#include <readline/readline.h> // Unix only, needs "libreadline-dev" installed to compile!
#include <readline/history.h>  // Unix only, needs "libreadline-dev" installed to compile!

using namespace std; // Makes my life easier

FTDIDevice device; // Device handle for the FTDI chip

void console()
{
	read_history(".uviemon_history"); // Load the history file
	rl_bind_key('\t', rl_complete);	  // Tab completion for readline

	while (true)
	{
		char *rawInput = nullptr;
		rawInput = readline("uviemon> "); // Read input from the user

		if (rawInput != nullptr) // Add the input to the readline history
		{
			const char *s = rawInput;
			while (*s != '\0')
			{
				if (!isspace((unsigned char)*s))
				{
					add_history(rawInput);
					break;
				}
				s++;
			}
		}

		string input(rawInput);
		stringstream ss(input);
		string word;
		string words[10]; // Assuming max of 10 words in input

		unsigned int inputWords = 0;
		while (ss >> word && inputWords < 10) // Get individual words in input and strip whitespace
		{
			words[inputWords] = word;
			inputWords++;
		}

		free(rawInput);

		if (inputWords == 0) // No input or just whitespace or enter presses
		{
			continue;
		}

		cout << endl;

		if (words[0] == "help") // List all avail commands
		{
			cout << "Usage:" << endl;
			cout << "  command <param#1> <param#2> ... <param#X>" << endl;
			cout << endl;
			cout << "List of commands:" << endl;
			cout << "  help: \t This list of all available commands" << endl;
			cout << "  scan: \t Scan for all possible IR opcodes" << endl;
			cout << endl;
			cout << "  mem: \t\t Read <length#2> 32-bit DWORDs from a starting <address#1> out of the memory" << endl;
			cout << "  memh: \t Read <length#2> 16-bit WORDs from a starting <address#1> out of the memory" << endl;
			cout << "  memb: \t Read <length#2> 8-bit BYTEs from a starting <address#1> out of the memory" << endl;
			cout << "  wmem: \t Write <data#2> 32-bit DWORD to a memory <address#1>" << endl;
			cout << "  wmemh: \t Write <data#2> 16-bit WORD to a memory <address#1>" << endl;
			cout << "  wmemb: \t Write <data#2> 8-bit BYTE to a memory <address#1>" << endl;
			cout << endl;
			cout << "  bdump: \t Read <length#2> BYTEs of data from memory starting at an <address#1>, saving the data to a <filePath#1>" << endl;
			cout << endl;
			cout << "  load: \t Write a file with <filePath#1> to the device memory" << endl;
			cout << "  verify: \t Verify a file written to the device memory with <filePath#1>" << endl;
			cout << "  wash: \t Wash memory with a certain DWORD <length#1> of hex DWORD <characters#3> starting at an <address#2>" << endl;
			cout << endl;
			cout << "  exit: \t Exit uviemon" << endl;
		}
		else if (words[0] == "scan")
		{
			BYTE irl = device.scanIRLength(); // Grab IR length, it's always (needs to be) 6 so I could skip this technically

			device.scanInstructionCodes(irl); // Debug function: Scan for the number of and all available opcodes on the device
		}
		else if (words[0] == "mem" || words[0] == "memh" || words[0] == "memb" || words[0] == "wmem" || words[0] == "wmemh" || words[0] == "wmemb")
		{
			if (inputWords >= 2)
			{
				DWORD addr;

				try // Check if user input is an integer
				{
					addr = stoul(words[1], nullptr, 16);
				}
				catch (invalid_argument const &e)
				{
					cerr << "Address is not a number!" << endl;
					continue;
				}
				catch (out_of_range const &e)
				{
					cerr << "Address is out of range for a DWORD!" << endl;
					continue;
				}

				if (inputWords == 2)
				{
					if (words[0] == "mem")
					{
						mem(device, addr);
					}
					else if (words[0] == "memh")
					{
						memh(device, addr);
					}
					else if (words[0] == "memb")
					{
						memb(device, addr);
					}
					else if (words[0] == "wmem" || words[0] == "wmemh" || words[0] == "wmemb") // Error!
					{
						cerr << "Missing argument <data#2>..." << endl;
					}
				}
				else if (inputWords >= 3)
				{
					if (words[0] == "mem" || words[0] == "memh" || words[0] == "memb")
					{
						DWORD length;

						try // Check if user input is an integer
						{
							length = stoul(words[2]);
						}
						catch (invalid_argument const &e)
						{
							cerr << "Length is not a number!" << endl;
							continue;
						}
						catch (out_of_range const &e)
						{
							cerr << "Length is out of range for a DWORD!" << endl;
							continue;
						}

						if (words[0] == "mem")
						{
							mem(device, addr, length);
						}
						else if (words[0] == "memh")
						{
							memh(device, addr, length);
						}
						else if (words[0] == "memb")
						{
							memb(device, addr, length);
						}
					}
					else if (words[0] == "wmem" || words[0] == "wmemh" || words[0] == "wmemb")
					{
						if (words[0] == "wmem")
						{
							DWORD data;

							try // Check if user input is an integer
							{
								data = stoul(words[2], nullptr, 16);
							}
							catch (invalid_argument const &e)
							{
								cerr << "Data is not a number!" << endl;
								continue;
							}
							catch (out_of_range const &e)
							{
								cerr << "Data is out of range for a DWORD!" << endl;
								continue;
							}

							wmem(device, addr, data);
						}
						else if (words[0] == "wmemh")
						{
							WORD data;

							try // Check if user input is an integer
							{
								data = stoul(words[2], nullptr, 16);
							}
							catch (invalid_argument const &e)
							{
								cerr << "Data is not a number!" << endl;
								continue;
							}
							catch (out_of_range const &e)
							{
								cerr << "Data is out of range for a WORD!" << endl;
								continue;
							}

							wmemh(device, addr, data);
						}
						else if (words[0] == "wmemb")
						{
							BYTE data;

							try // Check if user input is an integer
							{
								data = stoul(words[2], nullptr, 16);
							}
							catch (invalid_argument const &e)
							{
								cerr << "Data is not a number!" << endl;
								continue;
							}
							catch (out_of_range const &e)
							{
								cerr << "Data is out of range for a BYTE!" << endl;
								continue;
							}

							wmemb(device, addr, data);
						}
					}
				}
			}
			else
			{
				cerr << "Missing argument <address#1>..." << endl;
			}
		}
		else if (words[0] == "load")
		{
			if (inputWords >= 2)
			{
				load(device, words[1]);
			}
			else
			{
				cerr << "Missing argument <filePath#1>..." << endl;
			}
		}
		else if (words[0] == "verify")
		{
			if (inputWords >= 2)
			{
				verify(device, words[1]);
			}
			else
			{
				cerr << "Missing argument <filePath#1>..." << endl;
			}
		}
		else if (words[0] == "wash")
		{
			if (inputWords == 2)
			{
				DWORD length;

				try // Check if user input is an integer
				{
					length = stoul(words[1]);
				}
				catch (invalid_argument const &e)
				{
					cerr << "Length is not a number!" << endl;
					continue;
				}
				catch (out_of_range const &e)
				{
					cerr << "Length is out of range for a DWORD!" << endl;
					continue;
				}

				wash(device, length);
			}
			else if (inputWords == 3)
			{
				DWORD length, addr;

				try // Check if user input is an integer
				{
					length = stoul(words[1]);
					addr = stoul(words[2], nullptr, 16);
				}
				catch (invalid_argument const &e)
				{
					cerr << "Length/address is not a number!" << endl;
					continue;
				}
				catch (out_of_range const &e)
				{
					cerr << "Length/address is out of range for a DWORD!" << endl;
					continue;
				}

				wash(device, length, addr);
			}
			else if (inputWords >= 4)
			{
				DWORD length, addr, c;

				try // Check if user input is an integer
				{
					length = stoul(words[1]);
					addr = stoul(words[2], nullptr, 16);
					c = stoul(words[3], nullptr, 16);
				}
				catch (invalid_argument const &e)
				{
					cerr << "Length/address/character is not a number!" << endl;
					continue;
				}
				catch (out_of_range const &e)
				{
					cerr << "Length/address/character is out of range for a DWORD!" << endl;
					continue;
				}

				wash(device, length, addr, c);
			}
			else
			{
				wash(device);
			}
		}
		else if (words[0] == "bdump")
		{
			if (inputWords >= 4)
			{
				DWORD length, addr;

				try // Check if user input is an integer
				{
					length = stoul(words[2]);
					addr = stoul(words[1], nullptr, 16);
				}
				catch (invalid_argument const &e)
				{
					cerr << "Length/address is not a number!" << endl;
					continue;
				}
				catch (out_of_range const &e)
				{
					cerr << "Length/address is out of range for a DWORD!" << endl;
					continue;
				}

				bdump(device, addr, length, words[3]);
			}
			else
			{
				cerr << "Missing argument..." << endl;
			}
		}
		else if (words[0] == "run")
		{
			run(device);
		}
		else if (words[0] == "reset")
		{
			reset(device);
		}
		else if (words[0] == "exit")
		{
			break;
		}
		else
		{
			cout << "Command '" << input << "' not recognized. Type 'help' to get a list of commands." << endl;
		}
		cout << endl;
	}

	write_history(".uviemon_history"); // Save the history file
}

void showInfo()
{
	cout << "Replacement Tool for grmon used in SMILE mission debugging." << endl;
	cout << "March 2023 and later." << endl;
	cout << endl;

	FT_STATUS ftStatus;
	DWORD dwLibraryVer;

	// Get FTDI library version
	ftStatus = FT_GetLibraryVersion(&dwLibraryVer);
	if (ftStatus == FT_OK)
	{
		unsigned int majorVer = (dwLibraryVer >> 16) & 0xFF;
		unsigned int minorVer = (dwLibraryVer >> 8) & 0xFF;
		unsigned int buildVer = dwLibraryVer & 0xFF;

		cout << "FTDI library version: " << hex << majorVer << "." << minorVer << "." << buildVer << endl;
	}
	else
	{
		cout << "Error reading library version" << endl;
	}

	cout << "uviemon version: " << VERSION << endl;
	cout << endl;
}

void showHelp()
{
	cout << "Usage:" << endl;
	cout << endl;

	cout << "\t -help: \t This list of all available commands" << endl;
	cout << "\t -info: \t Version numbers and driver info" << endl;
	cout << "\t -list: \t List all available FTDI devices" << endl;
	cout << "\t -jtag <num>: \t Open console with jtag device" << endl;
	cout << endl;
}

int main(int argc, char *argv[])
{
	cout << "\n  ** uviemon v" << VERSION << " **\n"
		 << endl;
	cout << "  GR712RC Dual-Core LEON3FT SPARC V8 Processor debugging" << endl;
	cout << "  monitor using the FTDI FT2232H chipset for communication.\n"
		 << endl;

	if (argc < 2)
	{
		cerr << "Need a command to work!\n"
			 << endl;
		showHelp();
		return 1;
	}

	if (strcmp(argv[1], "-list") == 0)
	{
		device.getDeviceList();
	}
	else if (strcmp(argv[1], "-jtag") == 0)
	{
		if (argc < 3)
		{
			cerr << "Device index required for -jtag" << endl;
			return 1;
		}

		int deviceIndex = -1;

		try // Check if user input is an integer
		{
			deviceIndex = stoi(argv[2]);
		}
		catch (invalid_argument const &e)
		{
			cerr << "Input is not a number!" << endl;
			return 1;
		}
		catch (out_of_range const &e)
		{
			cerr << "Input is out of range for an integer!" << endl;
			return 1;
		}

		long int count = device.getDevicesCount(); // Get the number of attached FTDI devices

		if (deviceIndex < 0 || deviceIndex >= count)
		{
			cerr << "Device index cannot be smaller than 0 or larger than " << count << endl;
			return 1;
		}

		FT_STATUS ftStatus = device.open(deviceIndex); // Open the selected device and initialize all the settings

		if (ftStatus != FT_OK)
		{
			cerr << "Unable to use device " << deviceIndex << ". Aborting..." << endl;
			return 1;
		}

		cout << endl;
		unsigned int numberOfJTAGs = device.getJTAGCount(); // Get number of devices on the current JTAG chain

		if (numberOfJTAGs == 0) // Nothing found, maybe device doesn't have power?
		{
			cerr << "No devices connected on the JTAG chain! Exiting." << endl;
			return 1;
		}
		else if (numberOfJTAGs > 1)
		{
			cerr << "More than one device found on the JTAG chain. uviemon can only interface a single GR712!" << endl;
			return 1;
		}

		uint32_t id = device.readIDCODE();		  // Try to read IDCODE
		unsigned int irl = device.scanIRLength(); // Grab IR length

		if (irl != 6) // Must be a 6-bit IR, otherwise something's very wrong!
		{
			cerr << dec << irl << "-bit length, bad value!" << endl;
			cerr << "IR length is unequal to 6 bits. Can only work with the 6-bit GR712 IR! Exiting." << endl;
			return 1;
		}

		unsigned int length1 = device.scanDRLength(CODE_DATA); // Grab DR Data register length

		if (length1 != 33) // Must be a 33-bit DR, cannot work otherwise, everything's hard-coded
		{
			cerr << dec << length1 << "-bit length, bad value!" << endl;
			cerr << "Data register not working correctly. Need 33-bit GR712 register! Exiting." << endl;
			return 1;
		}

		unsigned int length2 = device.scanDRLength(CODE_ADDR_COMM); // Grab DR Command/Address register

		if (length2 != 35) // Must be a 35-bit DR, cannot work otherwise, everything's hard-coded
		{
			cerr << dec << length2 << "-bit length, bad value!" << endl;
			cerr << "Address/command register not working correctly. Need 35-bit GR712 register! Exiting." << endl;
			return 1;
		}

		cout << "Number of JTAG devices on chain: " << dec << numberOfJTAGs << endl;
		cout << "Device IDCODE: 0x" << hex << uppercase << id << endl;
		cout << "IR length: " << dec << irl << " bits" << endl;
		cout << "Data register length: 0x" << hex << uppercase << CODE_DATA << ", " << dec << length1 << " bits" << endl;
		cout << "Command/Address register length: 0x" << hex << uppercase << CODE_ADDR_COMM << ", " << dec << length2 << " bits" << endl;
		cout << "OK. Ready!" << endl;

		cout << endl;

		console();
	}
	else if (strcmp(argv[1], "-info") == 0)
	{
		showInfo();
	}
	else if (strcmp(argv[1], "-help") == 0)
	{
		showHelp();
	}
	else
	{
		cerr << "Unknown command '" << argv[1] << "'\n"
			 << endl;
		showHelp();

		return 1;
	}

	return 0;
}
