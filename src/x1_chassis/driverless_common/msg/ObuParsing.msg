std_msgs/Header header

# Scene data reporting
string device_id
int64 utc_time

float64[] position_3d
float64 latitude
float64 longitude
float64 elevation

float64[] motion
float64 speed
float64 heading
float64 steering_wheel_angle

float64[] vehicle_size
float64 width
float64 length
float64 height

# vehicle type
float64[] car_id
string car_id_infor
int8 car_type


# scenario type
string[] scenario_type


# scenario1-rsi
int8 rsi_alarmNum
ObuRsiData[] rsi_Data


# scenario2-spat
int8 spatAlarmType
int8 spat_priority

int8[] spat_advisorySpeed
int8 spat_minSpeed
int8 spat_maxSpeed

float64 spat_latitude
float64 spat_longitude
float64 spat_distance
int8 spat_curLaneId
int8 spat_direction
int8 spat_tscNum

int8[] spat_signalPhase
int8 spat_phaseId
int8 spat_waitTime
int8 spat_ledStatus

int8[] spat_laneAttributes
int8 spat_laneId
int8 spat_lane_direction


# scenario3-rsm
int8 rsm_alarmNum
ObuRsmData[] rsm_alarmData


# scenario4-bsm
int8 bsm_alarmNum
ObuBsmData[] bsm_alarmData


# scenario5-vslinformation
int8 overspeedStatus
int8 vsl_priority
int8 advisorySpeed


# scenario6-dataSharing
int8 ds_alarmNum
ObuSharingData[] ds_alarmData


# scenario7-object_sense
float64[] object_sense
int8 obj_num
int8 orient
float64[] obj_position
int8 obj_type_alarm
string obj_id
float64 obj_latitude
float64 obj_longitude

