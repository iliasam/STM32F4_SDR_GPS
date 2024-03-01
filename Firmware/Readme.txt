"project_single_sat" - example of using only one satellite. PRN is set in "main.c"
It is simplier, but has less comments.
This firmware can parse only "Subframe 1".
Current time is printed when "Subframe 1" has received.  

UART is not used here, "printf" is redirected to the SWO line - see compiler options.
Stack - 0x2000
Heap - 0x200

Compiler output:
30 642 bytes of readonly  code memory 
   220 bytes of readonly  data memory 
89 713 bytes of readwrite data memory 

Example of SWO output:
SAT=5
FINAL FREQ=1000 Hz
FINAL ACQ (SRCH_3) CODE=1990
START PRE_TRACK
FOUND TRACK PHASE=1991
PREAMBLE!
POLAR. FOUND
SUB ID=2
PREAMBLE!
SUB ID=3
PREAMBLE!
SUB ID=4
PREAMBLE!
SUB ID=5
PREAMBLE!
SUB ID=1
Curr. Time = Mon Nov 13 11:28:18 2023
PREAMBLE!
SUB ID=2

*********************

"project_main" - example of using 4 satellites, including localization. PRN is set in "main.c"

RTCM/Receiving state information is send to the UART, but "printf" is still redirected to the SWO line.

Stack - 0x2000
Heap - 0x1000 (needed for localization)

Compiler output:
 67 978 bytes of readonly  code memory 
    301 bytes of readonly  data memory 
105 958 bytes of readwrite data memory 

******************

IAR 7.50 is used here