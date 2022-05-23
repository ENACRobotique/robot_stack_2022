""" how to use
Use minus sign outside of the acceleration class -> Only used for "positive" acceleration
Use geogebra to check if slope are coherent or not
"""
class Acceleration():
    def __init__(self, max_speed, min_speed = 0.1, time_acceleration = 1.0, time_deceleration = 1.0, precision_error = 0.018, decelerate_before = 0.0): #~ 2 cm and ~ 1 deg
        #Decelerate before : the deceleration ramp will finish this value in meter before the target
        self.decelerate_before = decelerate_before
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.time_acceleration = time_acceleration #time to reach max speed from 0
        self.time_deceleration = time_deceleration #time to reach 0 from max speed
        accel_dist = 0.5 * self.time_acceleration * max_speed
        decel_dist = 0.5 * self.time_deceleration * max_speed
        self.accel_decel_dist = accel_dist + decel_dist
        self.precision_error = precision_error

        self.cur_time = 0
        self.top_speed_time = -1
        self.coeff_decel = -1000000000
        self.decel_b_coeff = -1

    def reset_accel(self):
        self.cur_time = 0
        self.top_speed_time = -1
        self.coeff_decel = -1000000000 #Value defined for "undefined - decelration not started yet"
        self.decel_b_coeff = -1

    def get_speed(self, cur_speed, distance_left, dt):
        
        speed_accel = self.cur_time * self.max_speed / self.time_acceleration

        self.cur_time += dt

        theoric_decel_dist = 0.5 * (self.time_deceleration + dt * 2) * cur_speed #+ dt  and + 0.1-> Margin for deceleration

        if cur_speed >= self.max_speed and self.top_speed_time == -1:
            self.top_speed_time = self.cur_time
            
        speed_decel = self.max_speed
        if distance_left <= theoric_decel_dist + self.decelerate_before: #adding a margin for deceleration
            if self.coeff_decel == -1000000000:
                #ax+b equation : need to find params a and b (a -> self.top_speed_time and solving b right now)
                self.coeff_decel = (- cur_speed)/ (self.time_deceleration)
                self.decel_b_coeff = - self.coeff_decel * (self.cur_time + self.time_deceleration) #b = - ax for time when stopped
                print(f"starting_decel_time {self.cur_time} after acceleration began")
            speed_decel = max (0, (self.coeff_decel * self.cur_time) + self.decel_b_coeff)
            #speed_decel = max(0, #manage both trapezoid and triangle not on top case
            #    - ((self.cur_time)/ self.time_deceleration - (self.start_decel_time - self.time_deceleration) / self.time_deceleration)
            #    )    
            if speed_decel == 0 and distance_left >= self.precision_error:
                speed_decel = self.min_speed
                print(f"undershooting target position - going at min speed {self.min_speed}")

        elif self.coeff_decel != -1000000000 and distance_left >= theoric_decel_dist:
            print("overshooting : distance_left >= theoric_decel_dist")

        return min(speed_accel, speed_decel)

if __name__ == "__main__":
    acc = Acceleration(max_speed = 1.0, min_speed = 0.1, time_acceleration = 2.0, time_deceleration = 0.5)
    dt = 0.05
    next_speed = 0
    distance_left = 1
    cpt = 0
    while distance_left >= 0.01 and cpt <= 1000:
        if cpt >= 400:
            pass #breakpoint place
        print(f"speed {next_speed} distance left {distance_left}")
        next_speed = acc.get_speed(next_speed, distance_left, dt)
        distance_left -= next_speed * dt
        cpt += 1