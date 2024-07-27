uint8 status
uint8 STATUS_NEW = 1
uint8 STATUS_UPDATE = 3
uint8 STATUS_COASTED = 4

uint8 label
uint8 LABEL_PERSON = 1
uint8 LABEL_CAR = 2
uint8 LABEL_TRUCK = 3

uint8 LABLE_UNKNOWN = 255

float32 azimuth   # left(+) right(-)
float32 distance  # m

float32 dis_x  # m
float32 dis_y  # m

# 相对速度, x向前, y向左 km/h
float32 speed_x
float32 speed_y

uint8 id
