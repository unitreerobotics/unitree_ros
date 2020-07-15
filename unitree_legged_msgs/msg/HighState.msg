uint8 levelFlag
uint16 commVersion
uint16 robotID
uint32 SN
uint8 bandWidth
uint8 mode
IMU imu
float32 forwardSpeed
float32 sideSpeed
float32 rotateSpeed
float32 bodyHeight
float32 updownSpeed
float32 forwardPosition       # (will be float type next version)
float32 sidePosition          # (will be float type next version)
Cartesian[4] footPosition2Body
Cartesian[4] footSpeed2Body
int16[4] footForce
int16[4] footForceEst    
uint32 tick               
uint8[40] wirelessRemote
uint32 reserve
uint32 crc

# Under are not defined in SDK yet.
Cartesian[4] eeForce            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.
float32[12] jointP              # for visualization