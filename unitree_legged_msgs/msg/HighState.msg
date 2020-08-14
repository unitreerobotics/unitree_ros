uint8 levelFlag
uint16 commVersion                  # Old version Aliengo does not have
uint16 robotID                      # Old version Aliengo does not have
uint32 SN                           # Old version Aliengo does not have
uint8 bandWidth                     # Old version Aliengo does not have
uint8 mode
IMU imu
float32 forwardSpeed
float32 sideSpeed
float32 rotateSpeed
float32 bodyHeight
float32 updownSpeed
float32 forwardPosition       # (will be float type next version)   # Old version Aliengo is different
float32 sidePosition          # (will be float type next version)   # Old version Aliengo is different
Cartesian[4] footPosition2Body
Cartesian[4] footSpeed2Body
int16[4] footForce                  # Old version Aliengo is different
int16[4] footForceEst               # Old version Aliengo does not have
uint32 tick               
uint8[40] wirelessRemote
uint32 reserve                      # Old version Aliengo does not have
uint32 crc

# Under are not defined in SDK yet. # Old version Aliengo does not have
Cartesian[4] eeForce            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.
float32[12] jointP              # for visualization