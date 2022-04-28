from ros2serial.conversions import send_diagnostic_data
#list all device and state machine possible to check if one is missing and raise diagnostic message if it's the case or detecting unknown things

capteurs = ['a4', 'a5', 'a6', 'a7', #AX-12
            's1', #servo
            'LR', #lecture de resistance

            #state machines : 
            'mv', #état state machine avant
            'mr', #état state machine arrière
            'hv', #contenu main avant
            'hr', #contenu main arrière
            'sc', #contenu réserve
    ]

#if pas reçu du code depuis x secondes sur l'état de tous ces capteurs : mettre un msg d'erreur
#if no missing_capteurs:
    #send_diagnostic_data(0, 'capteurs', 'Tous les capteurs sont connectés', 'stm32')
#else:
    #send_diagnostic_data(2), 'capteurs', 'Les capteurs suivants sont manquants : [capteur in missing_capteurs], 'stm32'
    #self.get_logger().info("""capteurs = ['a4', 'a5', 'a6', 'a7', #AX-12
    #        's1', #servo
    #        'LR', #lecture de resistance

    #        #state machines : 
    #        'mv', #état state machine avant
    #        'mr', #état state machine arrière
    #        'hv', #contenu main avant
    #        'hr', #contenu main arrière
    #        'sc', #contenu réserve
    #        """)