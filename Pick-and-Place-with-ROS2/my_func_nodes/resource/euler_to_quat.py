import numpy as np # Scientific computing library for Python
import math
 
def get_quaternion_from_rotation_vector(Rx, Ry, Rz):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """

  m = math.sqrt(Rx**2 + Ry**2 + Rz**2)


  qw = np.cos(m/2)
  
  if m < 0.001:
    qx = Rx/2
    qy = Ry/2
    qz = Rz/2
  else:
    qx = (Rx/m) * np.sin(m/2)
    qy = (Ry/m) * np.sin(m/2)
    qz = (Rz/m) * np.sin(m/2)
  
  print((m*180)/3.151592,m)
  return [qx, qy, qz, qw]
  
  
  
print(get_quaternion_from_rotation_vector(1.929, 2.473, -0.093))
