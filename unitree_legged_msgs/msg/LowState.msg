uint8 levelFlag
uint16 commVersion
uint16 robotID
uint32 SN 
uint8 bandWidth
IMU imu
MotorState[20] motorState
int16[4] footForce              # force sensors
int16[4] footForceEst           # force sensors
uint32 tick                     # reference real-time from motion controller (unit: us)
uint8[40] wirelessRemote        # wireless commands
uint32 reserve
uint32 crc


Cartesian[4] eeForceRaw
Cartesian[4] eeForce          #it's a 1-DOF force infact, but we use 3-DOF here just for visualization 
Cartesian position            # will delete
Cartesian velocity            # will delete
Cartesian velocity_w            # will delete
