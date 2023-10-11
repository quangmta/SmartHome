# Smart Home
Structure data Protocol (like Modbus): `Length (1 byte)`+`data (n bytes)` + `crc8 (1 byte)`

## Data received from server via UART

Data Structure (5 bytes): `prefix (1 byte)`+`value (4 bytes)`
		
'Prefix':
+ _"t"_: Temperature
+ _"v"_: Speed of fan
+ _"f"_: fan control
+ _"h"_: heater control
+ _"c"_: frequency converter control 
+ _"p"_: Kp
+ _"i"_: Ki
+ _"d"_: Kd
+ _"r"_: request


## Data sent to server

### Data of system:

Data Structure (10 bytes): `Prefix(symbol d) (1 byte)` + `Temperature (float 4 bytes)` + `Capacity of heater (float 4 bytes)` + `5 bits 0` + `Thermorstat (1 bit)` + `Relay (1 bit)` + `Frequency converter failure (1 bit)` 
   			        
### Acknowledgement of receiving data:

Data Structure (2 bytes): `Prefix (symbol a) (1 byte)`+`0 or 1 (1 byte)`			       