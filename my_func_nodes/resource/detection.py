import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

import depthai as dai

def get_distance_from_disparity(disparity_image, x, y):
    #Parámetros de la cámara -> importante calibracion
    baseline = 7.5 # en cm
    focal_length =  451.144 #calibrada en pixels
    
    # Calcula la distancia a partir de la disparidad
    disparity_value = disparity_image[y,x]
    distance = (baseline * focal_length) / disparity_value

    return distance


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
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

        self.timer = self.create_timer(5, self.actualizar_distance_depth) 

        self.depth_distance_obj = 0
        self.medidas = []

        #centro destino imagen de 8 bits
        self.dst_x = 0
        self.dst_y = 0

        self.bridge = CvBridge()
        
        self.centro_x_24bits = 0
        self.centro_y_24bits = 0

        # Distancia focal de la cámara RGB en metros
        self.focalLength = 0.00337

        # Altura real del objeto en cm
        self.realHeight = 0.074
   
    def actualizar_distance_depth(self):

        media = sum(self.medidas) / len(self.medidas)
        self.get_logger().info(f"EL objeto se encuentra a {self.depth_distance_obj} cm de distancia de la camara")

        self.medidas = []

    def def_depth(self, msg):
        #x, y = self.center.x, self.center.y
        if self.dst_x != 0 and self.dst_y != 0:
            x,y = self.dst_x, self.dst_y
        else:
            x, y = 320,200

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        depth_val = get_distance_from_disparity(cv_image,x,y)
        cv2.circle(cv_image, (x, y), 20, (255, 255, 255), 2)
        cv2.imshow('Depth measurement', cv_image)
        cv2.waitKey(3)

        #height, width = cv_image.shape
        #self.get_logger().info(f"imagen de 8 bits: {width} x {height}")

        if depth_val is not None:
            self.medidas.append(depth_val)

        self.depth_distance_obj = depth_val

        #libero la posicion para el siguiente objeto a coger
        self.dst_x = 0
        self.dst_y = 0
        
        

    def detect_object(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Definir el rango de valores HSV para el color verde
        green_lower = (30, 50, 50)
        green_upper = (70, 255, 255)

        black_lower = (0, 0, 0)
        black_upper = (180, 255, 30)

        white_lower = (0, 0, 231)
        white_upper = (180, 30, 255)

        # Aplicar una máscara binaria a la imagen HSV
        mask = cv2.inRange(hsv_image, white_lower, white_upper)
        filtered_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 50 and h > 50:  # Adjust these values according to your object size 50,50
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                #Coordenadas del objeto en la imgen a color de 24 bits
                self.centro_x_24bits = x + w/2
                self.centro_y_24bits = y + h/2
                #self.get_logger().info(f"Coordenadas centro.x = {self.centro_x_24bits} y centro.y = {self.centro_y_24bits}")
               
                #height, width, channels = cv_image.shape
                #self.get_logger().info(f"imagen de 24 bits: {width} x {height}")

                # Tamaño de la imagen de origen
                src_width = 1920
                src_height = 1080

                # Tamaño de la imagen de destino
                dst_width = 640
                dst_height = 400

                # Posición en la imagen de origen
                src_x = self.centro_x_24bits
                src_y = self.centro_y_24bits

                # Calcular la relación de escala
                scale_x = dst_width / src_width
                scale_y = dst_height / src_height

                # Aplicar la relación de escala a la posición en la imagen de origen
                self.dst_x = int(src_x * scale_x)
                self.dst_y = int(src_y * scale_y)



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
