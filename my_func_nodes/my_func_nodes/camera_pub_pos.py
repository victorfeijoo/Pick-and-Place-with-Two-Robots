import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np
import time

import depthai as dai

from geometry_msgs.msg import Pose, Point, Quaternion

#Disparity values from the disparity map function

def get_distance_from_disparity(disparity_image, x, y):
    #Camera parameters -> after calibration
    baseline = 7.5 #cm
    focal_length =  451.144 #on Pixels calibrated
    
    #Distance from disparity 
    disparity_value = disparity_image[y,x]
    distance = (baseline * focal_length) / disparity_value

    return distance


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        #Subscribers & Publishers

        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.detect_object,
            10)

        self.subs_disparity = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.def_depth,
            10)

        self.publisher_ = self.create_publisher(Pose, "object_position", 10)
        self.timer_ = self.create_timer(7, self.publish_news)

        self.subscriber_sec_colores = self.create_subscription(String, "sec_color", self.actualizar_sec_colores, 10) 
        
        self.quaternion = [0.570803357577419, 0.8205298265175397, -0.00518912143252199, 0.029789323459918908] #Fixed orientation

        self.timer = self.create_timer(4, self.actualizar_distance_depth) 

        self.depth_distance_obj = 0
        self.medidas = []

        #8 bits center dest
        self.dst_x = 0
        self.dst_y = 0

        self.bridge = CvBridge()
        
        self.centro_x_24bits = 0
        self.centro_y_24bits = 0

        self.position_x_to_robot = 0.00
        self.position_y_to_robot = 0.00
        self.position_z_to_robot = 0.00

        #Focal length of the RGB camera (m) 
        self.focalLength = 0.00337

        #Real height of the object (cm) 
        self.realHeight = 0.074

        self.mask_high = (30, 50, 50)
        self.mask_low = (70, 255, 255)

        self.counter = 0
        self.sec_color = ""

      
    #Colors function for changing the objective color of the object

    def actualizar_sec_colores(self, msg):
        self.sec_color = msg.data
        self.get_logger().info(f"funcion {self.sec_color}")
        self.timer = self.create_timer(25, self.change_color) 
        
    def change_color(self):
        self.counter = self.counter + 1

    #Publisher function for the robot control

    def publish_news(self):
 
        msg = Pose()
        
        msg.position.x = self.position_x_to_robot * 0.001
        msg.position.y = self.position_y_to_robot * 0.001 
        if self.position_z_to_robot > 0:
            if self.depth_distance_obj > 61:
                msg.position.z = 0.139 
            else:
                msg.position.z = float(self.position_z_to_robot) * 0.01
        else:
            msg.position.z = 0.139
        
        msg.orientation.x = self.quaternion[0]
        msg.orientation.y = self.quaternion[1]
        msg.orientation.z = self.quaternion[2]
        msg.orientation.w = self.quaternion[3]

        self.publisher_.publish(msg)
        
        self.get_logger().info("Mandando posicion {},{},{}".format(msg.position.x, msg.position.y, msg.position.z))

    #Depth update function
        
    def actualizar_distance_depth(self):

        self.get_logger().info(f"EL objeto se encuentra a {self.depth_distance_obj} cm de distancia de la camara")

        self.position_z_to_robot =  13 + (61.51 - (self.depth_distance_obj))  #61.51 from the floor, -13 because of the gripper. 
        self.medidas = []

    #Depth measurement function

    def def_depth(self, msg):
        #x, y = self.center.x, self.center.y
        if self.dst_x != 0 and self.dst_y != 0:
            x,y = self.dst_x, self.dst_y
        else:
            x, y = 320,200
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        depth_val = get_distance_from_disparity(cv_image,x,y)
        cv2.circle(cv_image, (x, y), 3, (255, 255, 255), 2)

        cv2.imshow('Depth measurement', cv_image)
        cv2.waitKey(3)

        if depth_val is not None:
            self.medidas.append(depth_val)

        self.depth_distance_obj = depth_val

        self.dst_x = 0
        self.dst_y = 0
        
    #Object detection of RGB 2D position for objects

    def detect_object(self, msg):

        #Format conversion
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)    

        #Calibration:
        """
        x2, y2 = 589, 367
        x1,y1= 1210, 367
                #height, width, channels = cv_image.shape
        cv2.circle(cv_image, (x2, y2), 4, (255, 255, 255), 2)
        cv2.circle(cv_image, (x1, y1), 4, (255, 255, 255), 2)
        distancia = abs(x2 - x1) + abs(y2 - y1)
        self.get_logger().info(f"distancia : {distancia}")
        """
        
        #Color ranges

        green_lower = (30, 100, 100)
        green_upper = (70, 255, 255)

        black_lower = (0, 0, 0)
        black_upper = (180, 255, 30)

        white_lower = (0, 0, 231)
        white_upper = (180, 30, 255)

        orange_lower = (10, 100, 20)
        orange_upper = (25, 255, 255)

        dark_blue_lower = (90, 50, 50)
        dark_blue_upper = (120, 255, 255)

        morado_lower = (135, 50, 50)
        morado_upper = (160, 255, 255)

        yellow_lower = (25, 100, 50)
        yellow_upper = (35, 255, 255)

        blue_lower = (90, 100, 90)
        blue_upper = (130, 255, 255)

        orange_lower = (0, 100, 100)
        orange_upper = (20, 255, 255)
        
        
        if len(self.sec_color) == 4 and self.counter < 5:
            
            if self.sec_color[self.counter] == "a":
                self.mask_high = blue_upper
                self.mask_low = blue_lower

            elif self.sec_color[self.counter] == "v":
                self.mask_high = green_upper
                self.mask_low = green_lower

            elif self.sec_color[self.counter] == "n":
                self.mask_high = orange_upper
                self.mask_low = orange_lower

            elif self.sec_color[self.counter] == "m":
                self.mask_high = morado_upper
                self.mask_low = morado_lower
            

        if(self.counter == len(self.sec_color)):
            self.counter = 0
        

        
        #Binary mask to HSV image 
        mask = cv2.inRange(hsv_image, self.mask_low , self.mask_high)
        filtered_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        #Contours recognition

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if (w > 60 and h > 100 and w < 100 and h < 250) or (h > 60 and w > 100 and h < 100 and w < 250):  #Values according to object size 
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                #Object coordinates of the object for 24 bits image 
                self.centro_x_24bits = x - w/2 - w/4#importante
                self.centro_y_24bits = y + h/2
                #self.get_logger().info(f"Coordenadas centro.x = {self.centro_x_24bits} y centro.y = {self.centro_y_24bits}")
                #self.get_logger().info(f"ancho del objeto: {w} y altura del objeto: {h}") #ancho 77 que son 24 mm y altura 216 que son 74 mm

                factor_escala = 25 / 621 #calibration

                #Size of the origin image 
                src_width = 1920
                src_height = 1080

                #Size of the destination image 
                dst_width = 640
                dst_height = 400

                #Position on the origin image 
                src_x = self.centro_x_24bits
                src_y = self.centro_y_24bits

                centro_cm_x = factor_escala * src_x
                centro_cm_y = factor_escala * src_y
                #self.get_logger().info(f"{centro_cm_x} , {centro_cm_y}")

                #References systems adjust
                centro_robot_cm_x = - 540 + centro_cm_x * 10
                centro_robot_cm_y = - 158 - centro_cm_y * 10 
                    
                #Scale relation calculation 
                scale_x = dst_width / src_width
                scale_y = dst_height / src_height

                #Application of scale between 8 and 24 bits images
                self.dst_x = int(src_x * scale_x)
                self.dst_y = int(src_y * scale_y)

                #self.get_logger().info(f"{self.dst_x},{self.dst_y}")
                
                #Position for the 8 bits image
                self.position_x_to_robot = float(centro_robot_cm_x)
                self.position_y_to_robot = float(centro_robot_cm_y)
            
        cv2.imshow('Object RGB Detector', cv_image)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
