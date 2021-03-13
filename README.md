ORBTrace Development
====================

This is the repository for the ORBTrace debug tool, targetting ARM CORTEX JTAG & SWD debug and  parallel TRACE, supporting Orbuculum.  This code was forked from Orbuculum on 13th March 2021. Change history prior to that point is contained in the Orbuculum repository. History starts afresh in this repository but, apart from documentation (this file), there is no discontinuity between the two.

This is built using Claire Wolfs' icestorm toolchain and currently targets a either a lattice iCE40HX-8K board or an Lambda Concept ECPIX-5 board (by default a -85F, but that's a trivial change in the makefile).

Trace is now complete (2nd February 2021). Work is now progressing on integrating JTAG and SWD to provide a complete debug probe. In theory, on an ECP5  or ICE40 HX8 part, it will work with any trace port operating up to at least 106MHz. Some stroking of the logic may make it a bit faster but there's no point doing that until everything else is done.  UP5K will run there up to around 50MHz, but if you're going out to buy something to use for running this code, that shouldn't be your first choice (ECPIX-5 should be, at least for now).

This should all be viewed as experimental. There remains work to be done....but see at the foot of this note. There are now two distinct families of builds; Verilog builds that build the whole stack and rely on the ftdi UART for communicaton, and nmigen builds that use an external ULPI for USB2-HS comms. You really want the nmigen USB2-HS version if you'e got a choice.

Note that ICE40HX1 support has been removed as it was just too snug to fit in the chip. The last version with (experimental) support for that chip can be found at https://github.com/orbcode/orbuculum/commit/58e7c03581231e0a37549060603aed7831c37533

Outstanding development actions;

 * Parse TPIU packets to split out ETM data and buffer it on-chip (will allow for post-mortem dumps)
 
Current testing status;

 * Tested on ICE40HX8K and ECPIX-5 at 1, 2 & 4 bit depths
 * Tested against Rising Edge and Falling edge synced flows
 * Tested with STM32F427 CPU running at 16MHz & 160MHz.
 * Tested with NRF5340 running at 32MHz

 * Needs testing at higher speeds and on more CPUs...this will be here forever, I guess

Verilog Builds
==============

This section is applicable to the ICE40HX8K, ICEBREAKER and ECPIX_5 boards.  It will result in comms over the ftdi link using either UART or SPI based communication. Note that SPI is faster, but only available on the ICE40.

To build it perform;

```
cd src
make ICE40HX8K_B_EVN

```
or;

```
cd src
make ECPIX_5_85F

```
or;
```
cd src
make ICEBREAKER
```


One the ICE40HX8 For normal operation you can burn the program image into the configuration serial memory
(J7:1-2, J6:2-4, and J6:1-3). For development just load it directly (J6:1-2 and
J6:3-4. Jumper J7 not installed). See Pg. 5 of the iCE40HX-8K Breakout Board User's Guide for
more information.

The ice40 breakout board is connected to the target via J2 as follows;

   * traceDin[0] C16
   * traceDin[1] D16
   * traceDin[2] E16
   * traceDin[3] F16
   * traceClk    H16

The ECPIX-5 is connected to the target via PMOD7 as follows;

   * traceDin[0] 0
   * traceDin[1] 1
   * traceDin[2] 2
   * traceDin[3] 3
   * traceClk    4

Make your grounds as good as possible...if you can twist a ground and clock together that will be helpful. Obviously you don't need the whole of traceDin[0..3] if you're only using 1 or 2 bit trace.

For ICE40 the system can be built using either an SPI or a UART transport layer. SPI is potentially capable of better performance (30MHz line rate). Testing shows that around 26Mbps is realistically acheivable. Both the serial and SPI interfaces are fully tested on ICE40.

For ECP5 only UART is supported at the moment since the hardware doesn't support the SPI...but you've also got the choice of ULPI for that.

Using UART data is presented at 12Mbaud over the serial port of the board. You can simply copy the frames of data to somewhere with something like this;

```
cat /dev/ttyUSB1 > myfile
```
Note that all syncs are removed from the data flow, which allows you to convey a lot more _real_ data. By default a sync is inserted every now and again, which keeps the UI sync'ed with the board.

Information on how to integrate it with orbuculum (hint, the `-o` option) is in the main README. Basically, you need to tell it that you're using the TRACE hardware and which serial port it appears on, like this;

```
ofiles/orbuculum -o 4 -p /dev/ttyUSB1
```

The value of the `-o` parameter sets the 'width' (i.e. number of bits) on the trace port hardware. If you include the `-m 100` kind of option it'll tell you how full the link is too. We sneak some other data into those sync packets from the board, so you will also see how many 16 byte frames of data have been received and the board LED status too, something like this;

```
1.5 MBits/sec (  51% full) LEDS: d--h Frames: 1903
```

The leds are `d`ata, `t`ransmit, `O`verflow and `h`eartbeat. Higher verbosity levels will report more link information, but that's mostly for debug and diagnostic purposes.

Once it's up and running you can treat it just like any other orbuculum data source. Note that you do need to set parallel trace, as opposed to SWO mode, in your setup. If you're using the gdbtrace.init file provided as part of orbuculum then this works fine for a STM32F427;

```
source ~/Develop/orbuculum/Support/gdbtrace.init

target remote localhost:2331
file ofiles/tracemule.elf
set mem inaccessible-by-default off
set print pretty
monitor reset
load
monitor reset

# This line for parallel trace output
enableSTM32TRACE 4

# For NRF53 use 'enableNRF53TRACE 4' instead

dwtSamplePC 1
dwtSyncTap 1
dwtPostTap 1
dwtPostInit 1
dwtPostReset 10
dwtCycEna 1
dwtTraceException 1

ITMId 1
ITMGTSFreq 0
ITMTSPrescale 3
ITMTXEna 1
ITMSYNCEna 1
ITMEna 1
ITMTSEna 0
ITMTER 0 0xFFFFFFFF
ITMTPR 0xFFFFFFFF
```

Note that signal integrity is a _huge_ issue. the `enableSTM32TRACE` command takes a second parameter which is drive strength, value 0-3. It defaults to 1. You may have to increase this.  If you have unexpected data corruption rest a finger on the TRACE input pins. If that fixes the issue, you know where it lies.  This will be addressed on 'production' hardware.

LEDs
====

 - ICE40:D2, ECPIX-5:LD5-Green: Heartbeat. Solid red when the bitfile is loaded. Flashes while the trace collector is running. (`h` in the `orbuculum` monitor)
 - ICE40:D4, ECPIX-5:LD6-Red: Overflow. Illuminated when there are too many data arriving over the trace link to be stored in local RAM prior to transmission (i.e. the off-board transmission link can't keep up), `O` in the monitor.
 - ICE40:D5, ECPIX-5:LD7-Blue: FallingEdge. (Diagnostic). Used to indicate that the link is synced to the falling edge rather than the rising one. Completely unimportant to end users, but very important to me...I will ask you for the state of this LED if you need support!
 - ICE40:D8, ECPIX-5:LD6-Green: Tx. Flashes while data is being sent over the serial link, `t` in the monitor.
 - ICE40:D9, ECPIX-5:LD8-Blue: Data. A 'sticky' version of D8 which will stay illuminated for about 0.7Secs after any data, `d` in the monitor.
 

ULPI Builds
===========

This is the preferred route since it offers much faster comms. At the moment it's only available on the ECPIX-5-85F board. To use it;

```
cd orbtrace/nmigen
python3 orbtrace_builder.py
```

...the image will be built and installed onto a connected ECPIX-5. By default it will burn the image to RAM. You will need to edit the build scripts to burn to flash (not advisable for now).

`orbuculum` will detect the Orbtrace board automatically, so no command line options are mandatory to use it.

IT IS VERY EXPERIMENTAL. CATS ARE AT RISK.
