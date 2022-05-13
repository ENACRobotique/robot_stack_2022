""" 
Workflow à tester:
Tester.launch_actuator_diagnostic()
Wait for ~1000 readline?
check result of each test
i c1 -> test capteur 1
[...]
i c100 -> test capteur 100
Send diagnostic depending on them


"""

"""

motor.init();
odom.init();
RUN_TEST(test_encoder_stop_1);
RUN_TEST(test_encoder_forward_1);
RUN_TEST(test_encoder_stop_1);
RUN_TEST(test_encoder_backward_1);
RUN_TEST(test_encoder_stop_1);

RUN_TEST(test_encoder_stop_2);
RUN_TEST(test_encoder_forward_2);
RUN_TEST(test_encoder_stop_2);
RUN_TEST(test_encoder_backward_2);
RUN_TEST(test_encoder_stop_2);

Faire une routine test actuateurs uniquement, faire une routine test fixe, faire une routine test mobile
"""

class Tester():
    def __init__(self) -> None:
        self.cur_batch_diagnostic_status = []

    def add_diagnostic_result(self, diagnostic_result:str):
        self.cur_batch_diagnostic_status.append()

    #Diagnostics array -> diagnostics_status[Encoder, Pumps,...]
    #keyvalues(Encoder) : Forward-left -> OK, FOrward-right -> OK,...