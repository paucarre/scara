import requests
import struct
import base64
import numpy as np
#base64_message = base64_bytes.decode('ascii')
#RobotState(linear_1=5, angle_1=to_radians(0.), angle_2=to_radians(70.), angle_3=to_radians(-20.))


def to_radians(degrees):
    return ( degrees * np.pi ) / 180.

def encode_double(data):
    data = bytearray(struct.pack("d", data))
    data = base64.b64encode(data)
    return data

linear_1=encode_double(5)
angle_1=encode_double(to_radians(0.))
angle_2=encode_double(to_radians(70.))
angle_3=encode_double(to_radians(-20.))
state = {
        'linear_1': linear_1,
        'angle_1': angle_1,
        'angle_2': angle_2,
        'angle_3': angle_3
    }
request = requests.post('http://192.168.0.39:7000/message', json=state)
print(request.status_code)
print(request.text)