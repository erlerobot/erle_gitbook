# MAVLink

MavLink es un protocolo de comunicación para MAV (Micro Aerial Vehicles) que hoy en día se ha extendido a todo tipo de aviones no tripulasdos (tanto aéreos como terrestres).

### Paquete MAVLink

El paquete MAVLink package es básicamente una secuencia de bytes codificados y enviados a través de un transductor (a través de serie USB, radio frecuencia, WiFi, GPRS, etc.). Mediante la codificación se ordena la información en un estructura de datos de *manera inteligente* añadiendo un **checksums (suma de control)**, **número de secuencia** y se envía a través del canal byte a byte.

#### Estructura del paquete

Cada paquete MAVLink esta estructurado de la siguiente manera:

![](../img/mavlinkROS/mavlink-packet.png)

La siguiente tabla describe las partes del mensajes y los bytes:

| **Partes del mensaje** | **Byte** | **Descripción** | **Valor** |
|------------------|----------|-----------------|-----------|
| Cabecera | 0 | Indica el inicio de un nuevo mensaje | v1.0: `0xFE` (v0.9: `0x55`) |
| Cabecera | 1 | Indica la longitud del contenido | 0 - 255 |
| Cabecera | 2 | Número de secuencia, de 0 a 255. Cada componente cuenta con una secuencia de envío. Permite detectar perdida de paquetes | 0 - 255 |
| Cabecera | 3 | ID del sistema de envío. Permite diferenciar diferentes MAVs en la misma red. | 1 - 255 |
| Cabecera | 4 | ID del componente emisor. Permite diferenciar diferentes componentes en el mismo sistema, por ejemplo. e IMU o el piloto automático | 0 - 255 |
| Cabecera | 5 | ID del mensaje (por ejemplo. 0 = heartbeat) - el id define que *signafica* el contenido y como debe ser decodificado. | 0 - 255 |
| *contenido* | 6 - (n + 6) | Contenido del mensaje | (0 - 255) bytes |
| **Checksum** | (n+7) to (n+8) | ITU X.25/SAE AS-4 hash, excluido el incio del mensaje, desde el byte 1..(n+6) Nota: El *checksum* también incluye `MAVLINK_CRC_EXTRA` (Número calculado a partir de campos del mensaje. Protege el paquete de ser decodificado de una versión diferente del mismo paquete pero con diferentes variables) | |

Algunos comentarios:
- La suma de control es la misma que utilizan los estandares ITU X.25 y SAE AS-4 (CRC-16-CCITT), documentados en in SAE AS5669A. Por favor, ver el códifo fuente de MAVLink para un ver un implementación comentada en C.
- La longitud mínima de paquete es de 8 bytes para un asentimiento sin contenido.
- La longitud máxima es de 263 bytes con el máximo contenido.

### Tipo de datos soportados
MAVLink soporta distintos tipos de enteros de tamaño fijo, IEEE 754 flotantes de precisión siempre, arrays de este tipo de dato (por ejemplo char[10]) y el campo especial `mavlink_version`, que es añadido automáticamante al protocolo. Estos tipos están disponibles:

- `char` - Characters / strings
- `uint8_t` - Unsigned 8 bit
- `int8_t` - Signed 8 bit
- `uint16_t` - Unsigned 16 bit
- `int16_t` - Signed 16 bit
- `uint32_t` - Unsigned 32 bit
- `int32_t` - Signed 32 bit
- `uint64_t` - Unsigned 64 bit
- `int64_t` - Signed 64 bit
- `float` - IEEE 754 single precision floating point number
- `double` - IEEE 754 double precision floating point number
- `uint8_t_mavlink_version` - Unsigned 8 bit El capo se rellena automáticamente con la versión actual de MAVLink - no puede ser escrito, solo se puede leer del paquete com un campo `uint8_t` normal.

### Rendimiento

Este protocolo fue totalemten orientado hacia dos propiedades: la velocidad y la seguridad de transmisión. Permite comprobar el contendio del mensaje, tmabién permite detectar la pérdida de mensajes, y solo neceista de 6 bytes de sobrecarga para cada paquete. Algunos ejemplos de transmisión se presentan a continuación:

| **velocidad de enlace** |	**Hardware** |	 **Tasa de refresco**|	 **Contenido**| **Valores *Float* **|
|----------------|---------------|------------------|---------------|-----------------|
|115200 baud    |   XBee Pro 2.4 GHz|	 50 Hz|	 224 bytes|	 56|
|115200 baud	| XBee Pro 2.4 GHz|	 100 Hz|	 109 bytes|	 27|
|57600 baud|	 XBee Pro 2.4 GHz|	 100 Hz|	 51 bytes|	 12|
|9600 baud|	 XBee Pro XSC 900|	 50 Hz|	 13 bytes|	 3|
|9600 baud|	 XBee Pro XSC 900|	 20 Hz|	 42 bytes|	 10|


### Fuentes:
- [MavLink Tutorial for Absolute Dummies (Part –I)](http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf)
- http://qgroundcontrol.org/mavlink/start
