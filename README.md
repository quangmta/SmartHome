# Smart Home
## Data received from server via UART

Structure: `prefix`+`value`+`\n`

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

For example: command `t25.5` sets the target temperature to 25.5 degrees, `f0` turns off the fan, `f1` turns on the fan.
## Data sent to server

Structure: `Temperature` + ` Capacity of heater` + ` Thermorstat` + ` Relay` + ` Frequency converter failure` + `\n`

For example: Command `26 2000 0 0 0` means the measure temperature is 26, the capacity of heater is 2000W, 
the heater, fan and frequency converter work normally.