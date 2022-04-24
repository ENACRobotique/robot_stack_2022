from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

"""
input synthax :
s       stop
a 1     advance for 1 sec
b 1     backward for 1 sec
l 180   turn left 180°
r 180   turn right 180°

"""

"""
Basic nav : Connu (x,y) -> position, unité Mètre + Orientation () [Message ros odometrie]
+ Destination (x,y) -> position, unité Mètre + Orientation () [//]

Recepetion : vitesse + position

Check : vitesse max : anticiper freinage 
Sortie : Tourner, puis ligne droite, orientation

2e niveau : Update de consignes : recommencer

https://pypi.org/project/simple-pid/ --> PID 
"""

"""
Basic navigation script : 
Input : take a destination position + rotation [ROS odometrie classic msg] + Max Speed
Result : Move Robot and stop at the right point, In case of update, stop robot and restart with new data
"""

def z_euler_from_quaternions(qx, qy, qz, qw):
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    return np.atan2(t3, t4)

class OdomData:
	x = 0.0
	y = 0.0
	rot_rad = 360

	previous_x = 0.0
	previous_y = 0.0
	previous_rot_rad = 360

	def __init__(self, x, y ,rot):
		self.x = x
		self.y = y
		self.rot = rot

	def update_position(self, x, y , rot):
		self.previous_x = self.x
		self.previous_y = self.y
		self.previous_rot_rad = self.rot_rad

		self.x = x
		self.y = y
		self.rot_rad = rot
	

#self.serial_send(CMD_VEL.format(int(vlin*1000), int(vtheta*1000)))


class Navigator(Node):
	_isNavigating = False
	_isRotating = False 
	
	_max_speed = 1.0 #Meter/seconds
	target = OdomData(0.0, 0.0, 6.28)
	current_position = OdomData(0.0, 0.0, 6.28)

	odom_topic = None
	velocity_publisher = None 

	rotationPrecision = 0.05 
	position_precision = 0.05

	#Stop point to be computed, to know when starting to stop (in regards of PID)
	stopPoint = OdomData(0.0, 0.0, 360)
	
	def __init__(self, xTarget, yTarget, rotTarget, maxSpeed):
		self.odom_topic = self.create_subscription(Odometry, "/odom", self.updatePosition, 10)

		self.target.x = xTarget
		self.target.y = yTarget
		self.target.rot = rotTarget

		self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
	
		self.stopPoint = stopLoc() #Thread it after successful tests	

	def updatePosition(self, msg):
		#Update position here
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		rot = z_euler_from_quaternions(msg.pose.pose.orientation.x,
        												msg.pose.pose.orientation.y,
        												msg.pose.pose.orientation.z,
        												msg.pose.pose.orientation.w)
		currentPosition.update_position(x, y, rot)

		if rot - target.rot > rotationPrecision : #meter
			self._isNavigating = False
			self._isRotating = True
			#Need Rotation
			self.rotate()
			return
		elif x - target.x <= position_precision and y - target.y <= position_precision :
			self._isNavigating = True
			self._isRotating = False
			self.move()
			return

		self._isNavigating = False
		self._isRotating = False
		
	def stop():
		target.x = current_position.x
		target.y = current_position.y
		target.rot = current_position.rot_rad
	
	def rotate():
		relative_rot_rad = current_position.rot_rad  - target.rot_rad

		speed = 0
		if (relative_rot_rad <= 0):
			speed = -2
		else:
			speed = 2
		
		msg = Twist()

		msg.twist.linear.x = 0.0
		msg.twist.linear.y = 0.0
		msg.twist.linear.z = 0.0
		msg.twist.angular.x = 0.0
		msg.twist.angular.y = 0.0
		msg.twist.angular.z = float(speed)

		self.velocity_publisher.publish(msg)






"""
/*
 * Trajectory.cpp
 *
 * Created on : 8 avril 2018
 * Author : Maxime
 */

#include "navigator.h"

#include "params.h"
#include "odometry.h"
#include "motorControl.h"
#include "math.h"
#include "utils.h"



Navigator navigator = Navigator();

Navigator::Navigator(){
	turn_done = False;
	displacement_done = False;
	trajectory_done = False;
	x_target = 0;
	y_target = 0;
	theta_target = 0;
	move_type = TURN;
	move_state = STOPPED;
}

void Navigator::move_to(float x, float y){
	odometry_motor.set_pos(odometry_wheel.get_pos_x(), odometry_wheel.get_pos_y(), odometry_wheel.get_pos_theta());
	SerialCtrl.println("odometry motor reset to odometry wheel position !");
	x_target = x;
	y_target = y;
	move_type = DISPLACEMENT;
	move_state = INITIAL_TURN;
	trajectory_done = False;
	SerialDebug.print("moving_to : ");
	SerialDebug.print(x_target);
	SerialDebug.print("\t");
	SerialDebug.println(y_target);
}//aa

void Navigator::move(float v, float omega){
	v_target = clamp(-SPEED_MAX, SPEED_MAX, v);
	omega_target = clamp(-OMEGA_MAX, OMEGA_MAX, omega);
	move_type = DISPLACEMENT;
	move_state = VELOCITY;
	trajectory_done = True;
	SerialDebug.print("velocity movement: ");
	SerialDebug.print(v_target);
	SerialDebug.print("\t");
	SerialDebug.println(omega_target);
}

void Navigator::step_forward(float d){
	move_to(d*cos(odometry_motor.get_pos_theta()) + odometry_motor.get_pos_x(),d*sin(odometry_motor.get_pos_theta()) + odometry_motor.get_pos_y());
}

void Navigator::step_backward(float d){
	move_to(-d*cos(odometry_motor.get_pos_theta()) + odometry_motor.get_pos_x(),-d*sin(odometry_motor.get_pos_theta()) + odometry_motor.get_pos_y());
}


void Navigator::turn_to(float theta){ // En degrés
	odometry_motor.set_pos(odometry_wheel.get_pos_x(), odometry_wheel.get_pos_y(), odometry_wheel.get_pos_theta());
	SerialCtrl.println("odometry motor reset to odometry wheel position !");

	theta_target = center_radian(PI*theta/180);

	/*SerialDebug.print("Angle: ");
	SerialDebug.println(odometry_motor.get_pos_theta());
	SerialDebug.print("moving_to : ");
	SerialDebug.print(theta_target);
	SerialDebug.print(" ( <  ");
	SerialDebug.println(theta);*/

	move_type = TURN;
	move_state = INITIAL_TURN;
	trajectory_done = False;
}

void Navigator::throw_to(float x, float y, float theta){
	x_target = x;
	y_target = y;
	theta_target = theta;
	move_type = THROW;
	move_state = CRUISE;
	trajectory_done = False;
	/*SerialDebug.print("throwing_to : ");
	SerialDebug.print(x_target);
	SerialDebug.print("\t");
	SerialDebug.println(y_target);*/
}

float Navigator::compute_cons_speed()
{
	float speed_cons, dist_fore, t_stop, dist_objective;
	int sgn,MAX_ACCEL;

	if(move_type == THROW){
		MAX_ACCEL = ACCEL_MAX_THROW;
	}
	else{
		MAX_ACCEL = ACCEL_MAX;
	}
	sgn = scalaire(cos(odometry_motor.get_pos_theta()),sin(odometry_motor.get_pos_theta()),x_target - odometry_motor.get_pos_x(),y_target - odometry_motor.get_pos_y());

	/*SerialDebug.print("Sens d'avancée:");
	SerialDebug.print("\t");
	SerialDebug.println(sgn);*/

	//Test de décélération (on suppose l'accélération minimale et on intègre deux fois)
	t_stop = odometry_motor.get_speed()/MAX_ACCEL;
	dist_fore = (odometry_motor.get_speed()*t_stop-1/2*MAX_ACCEL*pow(t_stop,2));
	/*dist_fore = odometry_motor.get_speed()*t_stop;*/

	dist_objective = sqrt(pow(x_target - odometry_motor.get_pos_x(),2) + pow(y_target - odometry_motor.get_pos_y(),2));

	//Si le point estimé est suffisamment proche du point voulu, on décélére, sinon on accélére jusqu'à la vitesse maximale.
	if(abs( dist_fore - dist_objective ) < ADMITTED_POSITION_ERROR){
		speed_cons = sgn*max(0,-MAX_ACCEL*NAVIGATOR_PERIOD + abs(odometry_motor.get_speed()));
	}
	else{
		if(dist_fore - dist_objective > 0){
			speed_cons = sgn*max(0,abs(odometry_motor.get_speed()) - MAX_ACCEL*NAVIGATOR_PERIOD);
		}
		else{
			speed_cons = sgn*min(SPEED_MAX,abs(odometry_motor.get_speed()) + MAX_ACCEL*NAVIGATOR_PERIOD);
		}
	} /*
	SerialDebug.print("Distances estimées");
	SerialDebug.print("\t");
	SerialDebug.print(dist_fore - dist_objective);
	SerialDebug.print("\t");
	SerialDebug.print(dist_objective);
	SerialDebug.print("speed cons : ");
	SerialDebug.print(speed_cons);
	SerialDebug.print("\tspeed= ");
	SerialDebug.println(odometry_motor.get_speed()); */
	return speed_cons;
}


float Navigator::compute_cons_omega()
{
	float omega_cons, angle_fore, alpha, t_rotation_stop;
	int sgn;

	if(move_type == DISPLACEMENT){
		alpha = odometry_motor.get_pos_theta() + center_axes(atan2((-y_target+odometry_motor.get_pos_y()),(-x_target+odometry_motor.get_pos_x())) - odometry_motor.get_pos_theta());
	}
	else{
		alpha = theta_target;
	}

	if (center_radian(alpha - odometry_motor.get_pos_theta()) > 0){
		sgn = 1;
	}
	else{
		sgn = -1;
	}
	t_rotation_stop = abs(odometry_motor.get_omega())/ACCEL_OMEGA_MAX;
	angle_fore = center_radian(odometry_motor.get_pos_theta() + sgn*(abs(odometry_motor.get_omega())*t_rotation_stop -1/2*ACCEL_OMEGA_MAX*pow(t_rotation_stop,2)));
	if(abs(center_radian(angle_fore - alpha)) < ADMITTED_ANGLE_ERROR){
		omega_cons = sgn*max(0,abs(odometry_motor.get_omega()) - NAVIGATOR_PERIOD*ACCEL_OMEGA_MAX);
	}
	else{
		if(sgn*(center_radian(alpha - angle_fore)) > 0){
			omega_cons = sgn*min(OMEGA_MAX, NAVIGATOR_PERIOD*ACCEL_OMEGA_MAX + abs(odometry_motor.get_omega()));
		}
		else{
			omega_cons = sgn*max(0,abs(odometry_motor.get_omega()) - NAVIGATOR_PERIOD*ACCEL_OMEGA_MAX);
		}
	} /*
	SerialDebug.print("Consigne angle:");
	SerialDebug.print(omega_cons);
	SerialDebug.print("\t");
	SerialDebug.print("Alpha:");
	SerialDebug.print(alpha);
	SerialDebug.print("\t");
	SerialDebug.print("angle_fore:");
	SerialDebug.println(angle_fore);
*/
	return omega_cons;
}

void Navigator::update(){
	float omega_cons,speed_cons,alpha,distance;

	if(move_type == BRAKE){
		int sgn = scalaire(cos(odometry_motor.get_pos_theta()),sin(odometry_motor.get_pos_theta()),x_target - odometry_motor.get_pos_x(),y_target - odometry_motor.get_pos_y());
		speed_cons = sgn*max(0,abs(odometry_motor.get_speed()) - EMERGENCY_BRAKE*NAVIGATOR_PERIOD);
		if(abs(odometry_motor.get_speed()) < ADMITTED_SPEED_ERROR){
			move_state = STOPPED;
			speed_cons = 0;
		}
		MotorControl::set_cons(speed_cons,0);
	}
	else{
		switch(move_state){
		case INITIAL_TURN:


			if(move_type==DISPLACEMENT){
				alpha = odometry_motor.get_pos_theta() + center_axes(atan2((-y_target+odometry_motor.get_pos_y()),(-x_target+odometry_motor.get_pos_x())) - odometry_motor.get_pos_theta());
			}
			else{
				alpha = theta_target;
			}
			turn_done = ((abs(center_radian(odometry_motor.get_pos_theta() - alpha)) < ADMITTED_ANGLE_ERROR)&&(odometry_motor.get_omega() < ADMITTED_OMEGA_ERROR));

			if(turn_done){

				MotorControl::set_cons(0,0);
				switch(move_type){
				case TURN:
					move_state = STOPPED;
					trajectory_done = True;
					break;
				case DISPLACEMENT:
					move_state = CRUISE;
					break;
				case THROW:
					//Do nothing
					break;
				case BRAKE:
					//Do nothing
					break;
				}
				break;
			}

			omega_cons = compute_cons_omega();
			MotorControl::set_cons(0,omega_cons);

			break;
		case CRUISE:
			distance = sqrt(pow(x_target - odometry_motor.get_pos_x(),2) + pow(y_target - odometry_motor.get_pos_y(),2));
			displacement_done = ((distance<ADMITTED_POSITION_ERROR)&&(odometry_motor.get_speed() < ADMITTED_SPEED_ERROR));
			if(displacement_done){
				MotorControl::set_cons(0,0);
				move_state=STOPPED;
				trajectory_done = True;
				break;
			}

			speed_cons=compute_cons_speed();
			omega_cons = compute_cons_omega();
			Serial.println("cruise mode : ");
			Serial.print("\t speed_cons :  ");
			Serial.print(speed_cons);
			Serial.print("\t omega_cons :  ");
			Serial.print(omega_cons);
			Serial.print("\t distance :  ");
			Serial.print(distance);
			Serial.println("***");
			MotorControl::set_cons(speed_cons,omega_cons);
			break;
		case STOPPED:
			//do nothing
			break;
		case VELOCITY:
			MotorControl::set_cons(v_target,omega_target);
			break;
		}
	}
}

void Navigator::forceStop(){
	move_type = BRAKE;
}

bool Navigator::moveForward(){
	int dir = scalaire(cos(odometry_motor.get_pos_theta()),sin(odometry_motor.get_pos_theta()),x_target - odometry_motor.get_pos_x(),y_target - odometry_motor.get_pos_y());
	if(dir>0){
		return True;
	}
	else{
		return False;
	}
}


float Navigator::center_axes(float angle)
{
	/*SerialDebug.print("center radian:");
		SerialDebug.print("\t");
		SerialDebug.print(angle);
		SerialDebug.print("\t");*/
	if (abs(angle) > PI){
		if(angle<0){
			while(abs(angle)>PI){
				angle+=PI*2;
			}
		}
		else{
			while(abs(angle)>PI){
				angle-=2*PI;
			}
		}
	}
	if(abs(angle+PI) + ADMITTED_ANGLE_ERROR < abs(angle)){
		angle+=PI;
	}
	if(abs(angle-PI) + ADMITTED_ANGLE_ERROR< abs(angle)){
		angle-=PI;
	}
	/*SerialDebug.println(angle);*/
	return angle;
}

float Navigator::center_radian(float angle)
{
	if (abs(angle) > PI){
		if(angle<0){
			while(abs(angle)>PI){
				angle+=PI*2;
			}
		}
		else{
			while(abs(angle)>PI){
				angle-=2*PI;
			}
		}
	}
	return angle;
}


int Navigator::scalaire(float x,float y,float x2,float y2){
	if(x*x2 + y*y2>0){
		return 1;
	}
	else{
		return -1;
	}
}

bool Navigator::isTrajectoryFinished(){
	return trajectory_done;
}

"""




