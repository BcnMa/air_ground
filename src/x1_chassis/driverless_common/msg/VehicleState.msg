# vehicle state reported from wired-chassis 

uint8 GEAR_INITIAL=0
uint8 GEAR_DRIVE=1
uint8 GEAR_NEUTRAL=2
uint8 GEAR_REVERSE=3
uint8 GEAR_PARKING=11

uint8   GEAR_INVALID = 200

bool    ready       # weather the vehicle is ready
bool    base_ready  # weather base system is ready for driverless
                    # false: driverless key is off, or emergency brake key is on, or other error in system

uint8   emergency_brake # bit0-bit7 represent 8 channels emergency stop signal
                        # bit7 represents software emergency
bool    driverless
uint8   gear
bool    hand_brake
float32 speed            #km/h
bool    speed_validity
float32 roadwheel_angle  #deg ccw(+) cw(-)
bool    roadwheel_angle_validity
bool    manualctrl_detected
float32 acceleration  # ±

uint8  accel_pedel_aperture  # 0-100  invalid(255)
uint8  brake_pedel_aperture  # 0-100  invalid(255)

## light
bool turnlight_l
bool turnlight_r
bool brake_light
bool low_beam
bool high_beam

bool horn

## state of charge
uint8 soc

## VCU_Version
string vcu_version



