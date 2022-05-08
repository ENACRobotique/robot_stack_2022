#used mainly for timestamp (don't forget the frame_id)
std_msgs/Header header

byte RECTANGLE=0
byte CIRCLE=1

#each index correspond to an unique obstacle
byte[] obstacle_type

float64[] width #in meter - use also only this one as radius for circle
float64[] height

geometry_msgs/Point[] position

