""" how to use
Use minus sign outside of the acceleration class -> Only used for "positive" acceleration
Use geogebra to check if slope are coherent or not
"""
class Acceleration():
    def __init__(self, max_speed, min_speed = 0.1, time_acceleration = 1.0, time_deceleration = 1.0):
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.time_acceleration = time_acceleration #time to reach max speed from 0
        self.time_deceleration = time_deceleration #time to reach 0 from max speed
        accel_dist = 0.5 * self.time_acceleration * max_speed
        decel_dist = 0.5 * self.time_deceleration * max_speed
        self.accel_decel_dist = accel_dist + decel_dist

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
        if distance_left <= theoric_decel_dist:
            if self.coeff_decel == -1000000000:
                #ax+b equation : need to find params a and b (a -> self.top_speed_time and solving b right now)
                self.coeff_decel = (- cur_speed)/ (self.time_deceleration)
                self.decel_b_coeff = - self.coeff_decel * (self.cur_time + self.time_deceleration) #b = - ax for time when stopped
                print(f"starting_decel_time {self.cur_time} after acceleration began")
            speed_decel = max (0, (self.coeff_decel * self.cur_time) + self.decel_b_coeff)
            #speed_decel = max(0, #manage both trapezoid and triangle not on top case
            #    - ((self.cur_time)/ self.time_deceleration - (self.start_decel_time - self.time_deceleration) / self.time_deceleration)
            #    )    
            pass
        elif self.coeff_decel != -1000000000 and distance_left >= theoric_decel_dist:
            print("overshooting : distance_left >= theoric_decel_dist")
        



        return min(speed_accel, speed_decel)




#TODO : prendre en compte les décélération trop tôt, ne pas décélérer entièrement ?
    def get_speed_2(self, cur_speed, distance_left, dt):
        #careful between max_speed and self.max_speed, not the same use case (maybe)
        self.cur_time += dt

        max_speed = self.max_speed
        #if distance_left <= self.accel_decel_dist: #lower max speed
        #    max_speed = distance_left/(2 * self.time_deceleration) #TODO : check if self.time_acceleration is better?
        #TODO : check if using a calculated live max_speed is better than using a constant max_speed when doing a full trapeze (and also when doing a triangle only)

        #normal acceleration trapezoid
        speed_accel = self.cur_time * self.max_speed / self.time_acceleration
        #time_at_max_speed = max(0, self.total_dist - self.accel_decel_dist)
        #time_at_max_speed = max(0, 
        #    (distance_left / self.max_speed) - self.accel_decel_dist
        #)
        future_speed = max(speed_accel, cur_speed) #if the speed_accel is higher, in case of triangle shape, the deceleration calculation is wrong using cur_speed, need to take into account accel_speed
        theoric_decel_dist = self.time_deceleration * (future_speed / max_speed) #self.max_speed ?
        if distance_left <= theoric_decel_dist: #slow down as much as possible to go back under the deceleration slope
            speed_decel = self.min_speed 
            print("overshooting - fast deceleration triggered")
        elif distance_left <= ((self.time_deceleration / max_speed) + dt): #normal deceleration trapezoid #adding dt to avoid rounding errors
            if self.start_decel_time == -1:
                self.start_decel_time = self.cur_time - dt #Adding dt margin ??
            speed_decel= max(0, 
            (-self.cur_time/self.time_deceleration)+(
                (self.time_acceleration + self.start_decel_time + self.time_deceleration)/ self.time_deceleration)  #theoric deceleration
        )
        else: #TODO : check if it's needed because also present in the return
            speed_decel = max_speed
        
        return min(speed_accel, speed_decel, max_speed)

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