class RobotParams():
    def __init__(self):
        self.topSpeed = 0
        self.minAccel = 0
        self.maxAccel = 0
        self.maxDecel = 0
        self.minDecel = 0


class TravelDistance():
    def __init__(self, params: RobotParams, topSpeed, distance):
        self.robotSpeed = 0
        self.params = params
        self.topSpeed = topSpeed
        self.distanceLeft = distance
        #cb to topics : add_distance_traveled & add_cur_speed

    def generate_setpoint(self):
        minBrakingDistance = self.params.maxDecel * self.robotSpeed #10% security margin
        maxBrakingDistance = self.params.maxDecel * self.robotSpeed * 1.1 #10% security margin TODO : peut-être une marge de 2 timescale?
        if self.distanceLeft >= maxBrakingDistance:
            return self.params.maxDecel
        elif self.distanceLeft >= minBrakingDistance: #(amax - amin)/(dmax-dmin)=(amax-?)/(dmax-dmin)
            #linear interpolation of deceleration rate
            decel = -((self.params.maxDecel - self.params.minDecel) / (maxBrakingDistance-minBrakingDistance))*(maxBrakingDistance-minBrakingDistance)-self.params.maxDecel
            return decel
        elif self.distanceLeft <= minBrakingDistance:
            return self.params.maxAccel #TODO : refaire mieux
        else:
            return 0

    def read_last_speed(self, speed: float):
        #timeframe
        timeframe = 1/30
        self.add_distance_traveled(speed * timeframe)
        self.add_cur_speed(speed)
        
    def add_distance_traveled(self, distance:float):
        self.distanceLeft -= distance

    def add_cur_speed(self, speed: float):
        self.robotSpeed = speed