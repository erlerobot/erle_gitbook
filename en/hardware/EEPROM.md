# Electrically Erasable Programmable Read Only Memory (EEPROM)


The robot Erle is equipped with a single **CAT24C256W** EEPROM to allow the Software to identify the board.

| Name | Size (bytes) | Contents |
| -----| -------------|----------|
| Header | 4 | `0xAA`, `0x55`, `0x33`, `0xEE` |
| Board Name | 8 | Name for board in **ASCII**: `A335BONE`|
| Version | 4 | Hardware version code for board in **ASCII**: `00A￼3` for Rev A3,`00A4` for Rev A4,`00A5` for Rev A5,`00A6` for Rev A6 |
| Serial Number | 12 | Serial number of the board. 12 character string is **ASCII** with the following scheme: `WWYY4P16nnnn`, where `WW` is a 2 digit week of the year of production, `YY` corresponds to the year of production, `nnnn` is an incrementing board number.|
| Configuration option | 32 | **ASCII**: `0000000000000000000000000000000` (64 0s in hex) |
| RSVD | 6 | **ASCII**: `000000` |
| RSVD | 6 | **ASCII**: `000000` |
| RSVD | 6 | **ASCII**: `000000` |
| Security | 20 | Board theft protection |
| Available | 32682 | Available for other non-volatile code/data |

