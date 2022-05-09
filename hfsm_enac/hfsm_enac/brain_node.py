#create a python ros node with name "brain"
class BrainNode(Node):
    def __init__(self):
        super().__init__('brain')

        #publisher for navigation msg and subscriber to peripherals topic and odometry
        self.nav_pub = self.create_publisher(Navigation, 'navigation')
        self.peripherals_sub = self.create_subscription(Peripherals, 'peripherals', self.check_state_machine, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.check_state_machine)

    def check_state_machine():
        pass
        