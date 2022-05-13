class Acceleration():
    def __init__(self, max_speed, time_acceleration = 1.0, time_deceleration = 1.0):
        self.max_speed = max_speed
        self.time_acceleration = time_acceleration #time to reach max speed from 0
        self.time_deceleration = time_deceleration #time to reach 0 from max speed
        accel_dist = 0.5 * self.time_acceleration * max_speed
        decel_dist = 0.5 * self.time_deceleration * max_speed
        self.accel_decel_dist = accel_dist + decel_dist

        self.cur_time = 0

    def reset_time(self):
        self.cur_time = 0

    def get_speed(self, distance_left, dt):
        self.cur_time += dt
        distanc
        speed_accel = self.cur_time * self.max_speed / self.time_acceleration
        speed_decel = min (
            - self.cur_time / self.time_deceleration + self.time_deceleration, #theoric deceleration
            #deceleration due to distance left
        )

        return min(speed_accel, speed_decel, self.max_speed)
