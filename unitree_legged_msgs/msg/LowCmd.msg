uint8 levelFlag                 
uint16 commVersion              # Old version Aliengo does not have
uint16 robotID                  # Old version Aliengo does not have
uint32 SN                       # Old version Aliengo does not have
uint8 bandWidth                 # Old version Aliengo does not have
MotorCmd[20] motorCmd
LED[4] led
uint8[40] wirelessRemote
uint32 reserve                  # Old version Aliengo does not have
uint32 crc

Cartesian[4] ff               # will delete # Old version Aliengo does not have