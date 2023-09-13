# uviemon

Replacement tool for grmon, low-level software debugging over JTAG via an FT2232H chip.

Used for debugging the GR712RC Dual-Core LEON3FT SPARC V8 Processor on the the Solar wind Magnetosphere Ionosphere Link Explorer, or SMILE -- a joint mission between the European Space Agency (ESA) and the Chinese Academy of Sciences (CAS). Anyways, probably can be used with any LEON SPARC V8 processors.

```text
g++ -o uviemon *.cpp *.c -L./lib/ftdi/build/ -lftd2xx -lreadline -Wall -O -pthread -std=c++14
```

**Uses git submodules for some of the included libraries!** After pulling this repo, don't forget to init and update all the submodules!

```text
git submodule update --init --recursive
```
