import threading
import serial

#class to manage the serial connection
class SerialManager():
    def __init__(self, port, timeout = 0.05, baudrate = 115200, rx_buffer_size=64, tx_buffer_size=64):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.ser.set_buffer_size(rx_buffer_size, tx_buffer_size)
        self.thread_read = threading.Thread(target=self.serial_read)
        self.listen = True

    def start_serial_read(self):
        self.thread_read.start()

    def stop_serial_read(self):
        self.listen = False

    def serial_send(self, msg):
        """Envoyer un message sur le port série"""
        self.ser.write(msg.encode('utf-8'))

    def serial_read(self):
        """Fonction qui lit en permanence le port série.
        S'exécute dans un thread séparé"""
        while self.listen:
            message = self.ser.readLine()
            message = message.decode()