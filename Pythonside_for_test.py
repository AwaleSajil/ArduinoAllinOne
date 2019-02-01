from __future__ import print_function
import sys
import serial
import glob
import struct
import time
import numpy as np



def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result



def get_time_millis():
    return(int(round(time.time() * 1000)))


def get_time_seconds():
    return(int(round(time.time() * 1000000)))



class ReadFromArduino(object):
    """A class to read the serial messages from Arduino. The code running on Arduino
    can for example be the ArduinoSide_LSM9DS0 sketch."""

    def __init__(self, port, SIZE_STRUCT = 8, verbose=0):
        self.port = port
        self.millis = get_time_millis()
        self.SIZE_STRUCT = SIZE_STRUCT
        self.verbose = verbose
        self.latest_values = -1
        self.t_init = get_time_millis()
        self.t = 0

        self.port.flushInput()

    def read_one_value(self):
        """Wait for next serial message from the Arduino, and read the whole
        message as a structure."""
        read = False

        while not read:
            myByte = self.port.read(1)
            if myByte.decode() == 'S':
                data = self.port.read(self.SIZE_STRUCT)
                myByte = self.port.read(1)
                if myByte.decode() == 'E':
                    self.t = (get_time_millis() - self.t_init) / 1000.0

                    # is  a valid message struct
                    new_values = struct.unpack('<hhhh', data)

                    current_time = get_time_millis()
                    time_elapsed = current_time - self.millis
                    self.millis = current_time

                    read = True

                    self.latest_values = np.array(new_values)

                    if self.verbose > 1:
                        print("Time elapsed since last (ms): " + str(time_elapsed))

                    return(True)

        return(False)

    def print_values(self):
        print("------ ONE MORE MEASUREMENT ------")
        print("Temperature: ", self.latest_values[0])
        print("Angular Humidity: ", self.latest_values[1])
        print("LDR: ", self.latest_values[2])
        print("Ammonia: ", self.latest_values[3])



# portno = look_for_available_ports();

# portno = serial_ports()
# print(portno)
# obj = ReadFromArduino(port = portno)


def sendValue(Data):
    Arduino.write(b'S')
    packed_data = struct.pack('>BBBB',Data[0], Data[1], Data[2], Data[3])
    Arduino.write(packed_data)
    Arduino.write(b'E')




class SendtoArduino(object):
    def __init__(self, port, SIZE_STRUCT = 4, verbose=0):
        self.port = port
        self.SIZE_STRUCT = SIZE_STRUCT
        self.verbose = verbose
        self.Data = np.array([1,2,3,4])



    def sendValue(self):
        Arduino.write(b'S')
        packed_data = struct.pack('>BBBB',self.Data[0], self.Data[1], self.Data[2], self.Data[3])
        Arduino.write(packed_data)
        Arduino.write(b'E')



# use is:
ports = serial_ports()
print(ports)
Arduino = serial.Serial(ports[0], baudrate=9600, timeout=0.5)
read_from_Arduino_instance = ReadFromArduino(Arduino, verbose=6)
send_to_Arduino_instance = SendtoArduino(Arduino, verbose=6)

while True:
    read_from_Arduino_instance.read_one_value()
    read_from_Arduino_instance.print_values()
    send_to_Arduino_instance.sendValue()
    time.sleep(0.5)

