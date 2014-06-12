# ADC

The processor has 8 ADC (Analog to Digital) converter inputs. The signals are 1.8V only interfaces. One of these, `AD7`, is connected to the PMIC (Power Management Integrated Circuit) TPS65217B and used for measuring voltages and current via the TPS65217B.

### ADC Inputs
The primary purpose of the ADC pins was intended for use as a Touchscreen controller but can be used as a general purpose ADC. Each signal is a 12 bit successive approximation register (SAR) ADC. Sample rate is 100K samples per second. There is only one ADC in the processor and it can be connected to any of the 8 ADC pins.

### VDD_ADC Interface
The signal `VDD_ADC` is provided via the expansion header, but is **not a voltage rail that is to be used to power anything on an expansion board**. It is supplied from the 1.8V rail of the TPS65217B and is run through an inductor for noise isolation. It is there if need for external circuitry to have access to the VREF rail of the ADC or to add additional filtering via a capacitor if needed.

### License

Part of the content is a derivative of the *BeagleBone System Reference Manual Rev A6.0.0*  licensed under the Creative Commons Attribution-Share Alike 3.0 Unported License. To view a copy of this license, visit [http://creativecommons.org/licenses/by-sa/3.0/](http://creativecommons.org/licenses/by-sa/3.0/) or send a letter to *Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA*.

All derivative works are to be attributed to Gerald Coley of BeagleBoard.org.

For any questions, concerns, or issues submit them to gerald@BeagleBoard.org.
