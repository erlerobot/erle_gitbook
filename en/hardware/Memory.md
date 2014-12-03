#Memory

A single 16 bit DDR2 memory device is used. The design supports 128MB or 256MB of memory. The standard configuration is 256MB at 400MHz.

The design uses a single MT47H128M16RT-25E:C 400MHZ memory from Micron which comes in an 84-Ball 9.0mm x 12.5mm FBGA package. The addressing configuration is shown in the following table:

| **Parameter** | **128 Meg x 16** |
|---------------|------------------|
| Configuration | 16 Meg x 16 x 8 banks|
| Refresh Count | 8K |
| Row Address | A[13:0] (16K)|
| Bank Address | BA[2:0] (8)|
| Column Address | A[9:0] (1K)|

The DDR2 connects direct to the processor and no external interface devices are required. Power is supplied to the DDR2 via the 1.8V rail on the TPS65217B.


###License

Part of the content is a derivative of the *BeagleBone System Reference Manual Rev A6.0.0*  licensed under the Creative Commons Attribution-Share Alike 3.0 Unported License. To view a copy of this license, visit [http://creativecommons.org/licenses/by-sa/3.0/](http://creativecommons.org/licenses/by-sa/3.0/) or send a letter to *Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA*.

All derivative works are to be attributed to Gerald Coley of BeagleBoard.org.

For any questions, concerns, or issues submit them to gerald@BeagleBoard.org.
