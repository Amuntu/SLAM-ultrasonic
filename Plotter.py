#Getting reading form Robot over a Serial cable:

import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import numpy as np

# Lists to store the x and y coordinates.
x_data_sensor_L = []
y_data_sensor_L = []

x_data_sensor_M = []
y_data_sensor_M = []

x_data_sensor_R = []
y_data_sensor_R = []

L = 175.0/1000.0 # Length from the center to the front tip of the triangle
W = 180.0/1000.0/ 2  # Half width of the base of the triangle
x0 = 0
y0 = 0
x_min = -2
x_max = 2
y_min = -2
y_max = 2

d_wheels2center = 75.0/1000.0 #Distance from the center of the wheels Base-Line to the center of the robot.

l_sensor_head = 6.0/1000.0 #length of the sensor speaker out of Robot in [m], 6mm --1/1000--> 0.006m
d_sensor_L = (105.5 / 1000.0) +  l_sensor_head #Distance from the center of the Robot to the left sensor's speaker.
d_sensor_M = (100.5 / 1000.0) +  l_sensor_head
d_sensor_R = (105.5 / 1000.0) +  l_sensor_head

theta_sensor_L =  44.0 * (math.pi / 180.0)
theta_sensor_M =  0.00 * (math.pi / 180.0)
theta_sensor_R = -44.0 * (math.pi / 180.0)

ser = serial.Serial('COM5', 115200)  # Replace 'COM5' & 115200 with the appropriate port for your system.
def is_valid_float(value):
    try:
        float(value)
        return True
    except ValueError:
        return False
def get_from_arduinoSerialPort():
    try:
        if ser.inWaiting() > 0:
            line = ser.read_until(b"\n").decode('latin-1').strip()
            ser.flushInput()
            if "," in line:
                values_str = line.split(",")
                #print(values_str)
                if len(values_str) == 6:
                  x = 0
                  y = 0
                  theta = 0
                  d1 = 0
                  d2 = 0
                  d3 = 0
                  if is_valid_float(values_str[0]) and is_valid_float(values_str[1]) and is_valid_float(values_str[2]):
                    x = float(values_str[0].strip())
                    y = float(values_str[1].strip())
                    theta = float(values_str[2].strip())*(math.pi / 180.0)
                  if is_valid_float(values_str[3]):
                    d1 = float(values_str[3].strip())/100.0
                  if is_valid_float(values_str[4]):
                    d2 = float(values_str[4].strip())/100.0
                  if is_valid_float(values_str[5]):
                    d3 = float(values_str[5].strip())/100.0
                    if d1 == 0:
                        d1 = None
                    if d2 == 0:
                        d2 = None
                    if d3 == 0:
                        d3 = None
                    return x, y, theta, d1, d2, d3
                else:
                    print("Unexpected number of values")
                    return None, None, None, None, None, None
            else:
                print("Unexpected response format")
                return None, None, None, None, None, None
        else:
            return None, None, None, None, None, None
    except Exception as e:
        print(f"Error: {e}")
        return None, None, None, None, None, None
def correctSensorsReadings_andPosition(x, y, theta, dL, dM, dR):
    x = x + d_wheels2center * math.cos(theta)
    y = y + d_wheels2center * math.sin(theta)
    if dL is not None:
       xs_L = x + (dL + d_sensor_L) * math.cos(theta + theta_sensor_L)
       ys_L = y + (dL + d_sensor_L) * math.sin(theta + theta_sensor_L)
    else:
       xs_L = None
       ys_L = None
    if dM is not None:
        xs_M = x + (dM + d_sensor_M) * math.cos(theta + theta_sensor_M)
        ys_M = y + (dM + d_sensor_M) * math.sin(theta + theta_sensor_M)
    else:
        xs_M = None
        ys_M = None
    if dR is not None:
        xs_R = x + (dR + d_sensor_R) * math.cos(theta + theta_sensor_R)
        ys_R = y + (dR + d_sensor_R) * math.sin(theta + theta_sensor_R)
    else:
        xs_R = None
        ys_R = None
    return x, y, xs_L, ys_L, xs_M, ys_M, xs_R, ys_R
def update(frame):
    plt.cla()
    plt.xlim(x_min, x_max)  # Set the x-axis limits
    plt.ylim(y_min, y_max)  # Set the y-axis limits
    x, y, theta, dL, dM, dR = get_from_arduinoSerialPort()
    #print( x, y, theta, dL, dM, dR)
    xs_L = None
    ys_L = None
    xs_M = None
    ys_M = None
    xs_R = None
    ys_R = None
    if x is not None and y is not None and theta is not None:
      x, y, xs_L, ys_L, xs_M, ys_M, xs_R, ys_R = correctSensorsReadings_andPosition(x, y, theta, dL, dM, dR)
      print(x, y, theta)
    if xs_L is not None and ys_L is not None:
        x_data_sensor_L.append(xs_L)
        y_data_sensor_L.append(ys_L)
        plt.scatter(x_data_sensor_L, y_data_sensor_L, c='g')
    if xs_M is not None and ys_M is not None:
        x_data_sensor_M.append(xs_M)
        y_data_sensor_M.append(ys_M)
        plt.scatter(x_data_sensor_M, y_data_sensor_M, c='y')
    if xs_R is not None and ys_R is not None:
        x_data_sensor_R.append(xs_R)
        y_data_sensor_R.append(ys_R)
        plt.scatter(x_data_sensor_R, y_data_sensor_R, c='b')
    if x is not None and y is not None and x > 0 and y > 0:
          # Triangle vertices in local coordinates (centered at the origin)
        triangle = np.array([
            [L, 0],          # Tip (front)
            [-L, W],         # Left base corner (back left)
            [-L, -W]         # Right base corner (back right)
        ])
    
        #Rotate the triangle to match the robot's orientation
        rotation_matrix  = np.array([
          [math.cos(theta), -math.sin(theta)],
          [math.sin(theta), math.cos(theta)]
        ])
        # Apply rotation to the triangle points
        rotated_triangle = triangle.dot(rotation_matrix)
        
        # Translate the triangle to the robot's position
        translated_triangle = rotated_triangle + np.array([x, y])
        # Plot the triangle
        plt.fill(translated_triangle[:, 0], translated_triangle[:, 1], color='blue')

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Real-time Ultrasonic Sensor Data')
    return plt

if __name__ == "__main__":
    fig = plt.figure()
    ani = animation.FuncAnimation(fig, update, interval=5)  # Update every 50 milliseconds
    plt.show()