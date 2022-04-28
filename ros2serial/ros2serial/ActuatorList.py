from ros2serial.conversions import send_diagnostic_data
#list all device and state machine possible to check if one is missing and raise diagnostic message if it's the case or detecting unknown things


class ActuatorList():
    def __init__(self, periph_check_timeout:int) -> None:  
        """makes sure that all sensors are detected and if not, raise error diagnostic message

        Args:
            periph_check_timeout (int): number of serial read before checking if all sensors were detected (-> count of each line read)
        """
        self.capteurs = ['a4', 'a5', 'a6', 'a7', #AX-12
                    's1', #servo
                    'LR', #lecture de resistance

                    #state machines : 
                    'mv', #état state machine avant
                    'mr', #état state machine arrière
                    'hv', #contenu main avant
                    'hr', #contenu main arrière
                    'sc', #contenu réserve
            ]
        self.detected_sensors = {}

        self.nb_periph_decl_since_reset = 0 #counter used to know when to trigger peripheral unconnected diagnostic
        self.nb_periph_decl_timeout = 150 #number of declaration before triggering unconnected diagnostic
    def sensor_list_check_missing(self, data:str):
        #check if the sensor is in the list and if not, raise warning diagnostic message
        if data in self.capteurs:
            self.detected_sensors[data] = True
        else:
            self.send_unknown_diagnostic_warn(data)
        self.nb_periph_decl_since_reset += 1
        if self.nb_periph_decl_since_reset == self.nb_periph_decl_timeout:
            is_missing_sensor = self.report_missing_sensor_error()
            if is_missing_sensor:
                #go back to the beggining of the check of missing sensor and if not, continue until serial disconnection
                self.nb_periph_decl_since_reset = 0 

    def report_missing_sensor_error(self)->bool:
        #check if all sensors are detected and if not, raise error diagnostic message
        #and return True if error detected
        missing_capteurs = self.capteurs.copy()
        for capteur in self.detected_sensors:
            missing_capteurs.remove(capteur)
        if len(missing_capteurs) == 0:
            send_diagnostic_data(0, 'capteurs', 'Tous les capteurs sont connectés', 'stm32')
        else:
            send_diagnostic_data(2, 'capteurs', f'Les capteurs suivants sont manquants : {missing_capteurs}', 'stm32')
            self.get_logger().info("""capteurs = ['a4', 'a5', 'a6', 'a7', #AX-12
                's1', #servo
                'LR', #lecture de resistance
                #state machines : 
                'mv', #état state machine avant
                'mr', #état state machine arrière
                'hv', #contenu main avant
                'hr', #contenu main arrière
                'sc', #contenu réserve
                """)

    def send_unknown_diagnostic_warn(self, name:str):
        send_diagnostic_data(1, "capteurs - inconnu", f"error : un capteur inconnu a été inséré - {name}", "stm32")
