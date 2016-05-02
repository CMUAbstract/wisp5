# libwispbase: interface to system and peripherals on WISP

This repo is a fork of wisp/wisp5 repo, but with all else purged except the
wisp-base library. The library was then packaged into a Maker package, for
easy inclusion as a dependency.

The future of this library and this repo is uncertain: libwispbase should be
factored into smaller components. The components would be generic for MSP430,
not limited to the WISP platform, e.g. consider the UART code (note that the
existing libmspconsole lib is not that vision, because it thinly wraps
libwispbase). At that point it might be cleaner to create new repos that are
not literal forks of wisp/wisp5 repo but are "logical forks" through citation.

Original README from the wisp/wisp5 repo follows:



WISP 5
====

Welcome to the WISP5 firmware repository!

Got questions? Check out the tutorials and discussion board at: http://wisp5.wikispaces.com

Schematics for the WISP5 prototype are temporarily available here: 
http://sensor.cs.washington.edu/wisp5/wisp5-schem.pdf

Interested in building a host-side application to talk with WISPs? Look no further than the SLLURP library for configuring LLRP-based RFID readers:
https://github.com/ransford/sllurp

Important Notices
----
Please note that the MSP430FR5969 included on the WISP 5 is not compatible with TI Code Composer Studio versions prior to version 6. Please use CCS v6 or above.

The WISP 5 is intended to be compatible with Impinj Speedway and Impinj Speedway Revolution series FCC-compliant readers. For updates about compatibility with other readers, please contact the developers.

Configuration
----
1. Set your Code Composer Studio v6x workspace to wisp5/ccs and import the following projects:

 * **wisp-base** The standard library for the WISP5. Compiled as a static library.
 * **run-once** An application which generates a table of random numbers and stores them to non-volatile memory on the WISP, for use in slotting protocol and unique identification.

 * **simpleAckDemo** An application which references wisp-base and demonstrates basic communication with an RFID reader.

 * **accelDemo** An application which references wisp-base and demonstrates sampling of the accelerometer and returning acceleration data to the reader through fields of the EPC.

2. Build wisp-base and then the two applications.

3. Program and run your WISP5 with run-once, and wait for LED to pulse to indicate completion.

4. Program and run your WISP5 with simpleAckDemo and ensure that it can communicate with the reader. Use an Impinj Speedway series reader with Tari = 6.25us or 7.14us, link frequency = 640kHz, and reverse modulation type = FM0.

A summary of protocol details is given below.

Protocol summary
----

Delimiter = 12.5us

Tari = 6.25us

Link Frequency (T=>R) = 640kHz

Divide Ratio (DR) = 64/3

Reverse modulation type = FM0

RTCal (R=>T) = Nominally 15.625us (2.5*Data-0), Appears to accept 12.5us to 18.75us

TRCal (R=>T) = Appears to accept 13.75us to 56.25us, reader usage of this field may vary.

Data-0 (R=>T) = 6.25us

PW (R=>T) = 3.125us (0.5*(Data-0))

Enjoy the WISP5, and please contribute your comments and bugfixes here!


