This is a ESP32 project that has been tested on XIAO_ESP32S3 to work with AD5940 EVAL board. The build tool was `arduino-cli`. A SqrWaveVoltammetry scan was executed with this firmware and the data was transmitted successfully.

NOTE: The GND pin right next to the SPI pin group the on AD5940 EVAL board is the only real GND that you should use. Looking into the design layout file, some pins are labeled as GND/AGND but they are not connected to anything on the eval board.

Pins to use on AD5940 EVAL board (-> pinName means connected to the pin with the silkscreen pinName on XIAO_ESP32S3):

| Arduino UNO 8/6 Pin     | Arduino UNO 10/8 Pin   |
| ----------------------- | ---------------------- |
|                         |                        |
|                         |                        |
|                         |                        |
| red (3v3) -> 3V3        | black (GND) -> GND     |
|                         | purple (SPI_CLK) -> D0 |
| _Fake GND/AGND_         | grey (SPI_MISO) -> D1  |
| _Fake GND/AGND_         | blue (SPI_MOSI) -> D2  |
|                         | white (CS) -> D3       |
| XXXXXXXX                |                        |
|                         |                        |
|                         | XXXXXXXX               |
|                         |                        |
| orange (594X_Rst) -> D4 |                        |
|                         |                        |
|                         |                        |
| XXXXXXXX                |                        |
| XXXXXXXX                | red (Interrupt) -> D5  |
| XXXXXXXX                |                        |
| XXXXXXXX                |                        |
