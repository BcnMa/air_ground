
uint8 GEAR_INITIAL=0
uint8 GEAR_DRIVE=1
uint8 GEAR_NEUTRAL=2
uint8 GEAR_REVERSE=3
uint8 GEAR_PARKING=11

uint8 BRAKE_MODE_DECELERATION=0
uint8 BRAKE_MODE_PEDAL_APERTURE=1

uint8 DRIVE_MODE_AUTO=0
uint8 DRIVE_MODE_REMOTE=1

uint8   drive_mode
bool    driverless
uint8   gear
float32 speed       #km/h

uint8 brake_mode
uint8 brake_pedel_aperture  #% 0-100
float32 deceleration  #m/s2  >=0

float32 acceleration  #m/s2  >=0
float32 roadwheel_angle  #deg ccw(+) cw(-)

bool    emergency_brake
bool    hand_brake
bool    turnlight_r
bool    turnlight_l
bool    low_beam
bool    high_beam
bool    brake_light
bool    horn
