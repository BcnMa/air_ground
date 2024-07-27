std_msgs/Header header

# car info
string carNum
int8 car_type

int8[] gspa
int8 gspaFlag

int8 platooningFlag
int8 platooningNum
string platooningId

float64[] leaderJoin
int8 isAgreeJoin
string lj_vid

float64[] leaderLeave
int8 leaveFlag
string ll_vid


# location info
float64 latitude
float64 longitude
float64 heading
int64 timestamp
float64 speed

float64[] carInfo
uint8 brakePedalStatus
uint8 turnLeftStatus
uint8 turnRightStatus
uint8 hazardStatus
uint8 absStatus
uint8 parkingLightStatus
float64 steeringAngle