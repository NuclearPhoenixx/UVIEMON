# uviemon

Replacement tool for grmon, low-level software debugging over JTAG via an FT2232H chip.

Used for debugging the GR712RC Dual-Core LEON3FT SPARC V8 Processor on the the Solar wind Magnetosphere Ionosphere Link Explorer, or SMILE -- a joint mission between the European Space Agency (ESA) and the Chinese Academy of Sciences (CAS).

```text
g++ -o uviemon *.cpp *.c -L./lib/ftdi/build/ -lftd2xx -lreadline -Wall -O -pthread -std=c++11
```

**TODO:** Use git submodules for include libs ([aluntzer/flightos](https://github.com/aluntzer/flightos))

**TODO:** All the rest
