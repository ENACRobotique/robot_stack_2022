class Acceleration():
    def __init__(self, max_speed, min_speed=0.05, time_acceleration=1.0, time_deceleration=1.0):
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.time_acceleration = time_acceleration  # time to reach max speed from 0
        self.time_deceleration = time_deceleration  # time to reach 0 from max speed
        accel_dist = 0.5 * self.time_acceleration * max_speed
        self.decel_dist = 0.5 * self.time_deceleration * max_speed
        self.accel_decel_dist = accel_dist + decel_dist

        self.cur_time = 0

    def reset_time(self):
        self.cur_time = 0

    def get_speed(self, distance_left, dt):
        self.cur_time += dt
        if self.decel_dist <= distance_left:
            # if robot is too fast to slow down on time for target without overshooting, slow down to minimal speed
            speed_decel = self.min_speed
        else:
            speed_decel = - self.cur_time / self.time_deceleration + \
                self.time_deceleration,  # theoric deceleration

        speed_accel = self.cur_time * self.max_speed / self.time_acceleration

        return min(speed_accel, speed_decel, self.max_speed)
