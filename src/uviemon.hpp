/*
	========================================
	uviemon: free(TM) replacement for grmon

	This is the main routine running when
	uviemon gets executed. It consists of
	the command line params and the console.
	========================================

	Compat:
		bee on/off
		grmon -jtaglist -ftdi						--> uviemon -list
		grmon -ftdi -u -ftdifreq 2 -jtagcable 3		--> uviemon -jtag 2
		grmon --help								--> uviemon -help


	ISSUE: ISO C++ forbids variable length array
*/

#include "ftdi_device.hpp"
#include "uviemon_cli.hpp"

#define VERSION "1.0.0" // Define some version number

/*
	Define a new object that will be used to connect to the FTDI chip and do all the JTAG comms
	"extern" to use for the FTDIDeviceWrapper.
*/
extern FTDIDevice device;

void console();
void showInfo();
void showHelp();

int main(int argc, char *argv[]);
