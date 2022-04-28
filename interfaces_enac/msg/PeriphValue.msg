#this message contain data for one of the actuators or sensor
byte WRITE=0
byte READ=1
byte SPAM=2
byte DECLARATION=3

#header -> Contain the origin and the stamp
#frame_id inside header-> value are not Transform Frame but device that sent the msg (stm32, raspy,..)
std_msgs/Header header

#Can be used to confirm if the origin and the type of operation makes sense for the current actuator
byte operation_type

#name of the actuator/sensor that will be used
string periph_name

int32 value

#can be used to confirm if the operation is correct (can be rad, rad/s, m, m/s, ...)
string unit