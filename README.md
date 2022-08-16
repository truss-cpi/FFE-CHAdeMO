# FFE-CHAdeMO
Code and info for after-market CHAdeMO modifications to the Ford Focus Electric

The data provided has not been vetted or officially validated in any way. It is provided with no warranty and is used entirely at your own risk. 

High Voltage DC can easily maim or kill and should only be worked on by appropriately trained professionals.

You assume all responsibility if you damage anything or hurt/kill anyone using this code or information - and you agree that the creator of this repository will not be held liable in any way.

Remember: Not only will this kill you, it will hurt the whole time you're dying.

UPDATE:
A redesign of the control circuit is under way using parts that are more readily attainable and that do not require surface mount soldering. The ADM3052 is getting replaced by an MCP2561 High Speed CAN Transceiver, with SFH636 High Speed (1MBit) Optocouplers carrying the CAN0 Rx/Tx between the two isolated sections of the shield. These couplers are good for 2MHz switching frequency and the 500kbit data rate we're using should only need half that.

Validation is in progress with a "test" version of the shield code that can verify inputs/outputs, relay control and CAN transmission.

It should be noted that if the CAN1RX pin (DAC0) is not tied to ground during validation testing, the CAN1TX pin (53) will not output test data reliably.

New schematics to follow soon followed by a new board layout.
