# Memoria programable de sólo lectura (EEPROM)

El robot Erle está equipado con una sola **CAT24C256W** EEPROM para permitir que el software pueda identificar la tarjeta.

| Nombre | Tamaño ( bytes) | Contenido |
| ----- | ------------- | ---------- |
| Cabecera | 4 | ` 0xAA `, ` 0x55 `, ` 0x33 `, ` 0xEE son ` |
| Nombre de la tarjeta | 8 | Nombre para la tarjeta en **ASCII**: ` A335BONE ` |
| Versión | 4 | Código Versión de hardware para la tarjeta en **ASCII**: ` ¿ 00Aï Œ 3 ` para Rev A3, ` 00A4 ` para Rev A4, ` 00A5 ` para Rev A5, ` 00A6 ` para Rev A6 |
| Número de serie | 12 | Número de serie de la placa. 12 cadena de caracteres **ASCII** con el siguiente esquema: ` WWYY4P16nnnn ` , donde ` WW ` es una semana de 2 dígitos del año de fabricación, ` YY ` se corresponde con el año de fabricación, ` nnnn`  es un tablero de incrementar número. |
| La opción de configuración | 32 | **ASCII**: ` 0000000000000000000000000000000 ` ( 64 0s en hexadecimal) |
| RSVD | 6 | **ASCII**: ` 000.000 ` |
| RSVD | 6 | **ASCII**: ` 000.000 ` |
| RSVD | 6 | **ASCII**: ` 000.000 ` |
| Seguridad | 20 | protección de robo de la Junta |
| Disponible | 32682 | Disponible para otro código / datos no volátil |
