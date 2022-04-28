class NavSupervisor():
    def __init__(self, stamp_callback) -> None:
        self.get_last_stamp = stamp_callback
        self.last_stamp = self.get_last_stamp()
        pass

    def update(self):
        """ check if the last message cmd_vel is older than 1s 
        and send a stop command if that's the case """
        if self.get_last_stamp() - self.last_stamp >= 1.0:
            self.serial_send(CMD_STOP)
            self.last_stamp = self.get_last_stamp()

    def on_ros_cmd_vel(self, msg):
        vlin = msg.linear.x
        vtheta = msg.angular.z
        self.serial_send(CMD_VEL.format(int(vlin*1000), int(vtheta*1000)))