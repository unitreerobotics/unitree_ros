uint8 levelFlag
uint16 commVersion              # Old version Aliengo does not have
uint16 robotID                  # Old version Aliengo does not have
uint32 SN                       # Old version Aliengo does not have
uint8 bandWidth                 # Old version Aliengo does not have
IMU imu
MotorState[20] motorState
int16[4] footForce              # force sensors     # Old version Aliengo is different
int16[4] footForceEst           # force sensors     # Old version Aliengo does not have
uint32 tick                     # reference real-time from motion controller (unit: us)
uint8[40] wirelessRemote        # wireless commands
uint32 reserve                  # Old version Aliengo does not have
uint32 crc

# Old version Aliengo does not have:
Cartesian[4] eeForceRaw
Cartesian[4] eeForce          #it's a 1-DOF force infact, but we use 3-DOF here just for visualization 
Cartesian position            # will delete
Cartesian velocity            # will delete
Cartesian velocity_w            # will delete
