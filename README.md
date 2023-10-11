# Smart Home
Structure data Protocol (like Modbus):  `Length`+`data` +   `crc8`

									     1 byte + n byte + 1 bytes
## Data received from server via UART

Data Structure (5 bytes): `prefix`+`value`

				1 byte + 4 byte 
				
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

Data Structure (10 bytes): 'Prefix(d)` + `Temperature` + `Capacity of heater` + `Thermorstat` + `Relay` + `Frequency converter failure` 

					        1 byte +   float 4 byte +	 float 4 bytes     + 5 bits + 1 bit +  1 bit +    1 bit
					        
### Acknowledgement of receiving data:

Data Structure (2 bytes): `Prefix (a)`+`0 or 1`
							1 byte    + 1 byte				       