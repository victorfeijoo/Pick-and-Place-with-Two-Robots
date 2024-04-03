import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import sys
import os
import cv2
import time

import depthai as dai

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Creation of pipeline
        self.pipeline = dai.Pipeline()

        # Creation of processing nodes I/O
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.colorCam = self.pipeline.create(dai.node.ColorCamera)
        self.depth = self.pipeline.create(dai.node.StereoDepth)

        self.xout = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)

        #Stream

        self.xout.setStreamName("disparity")
        self.xoutRgb.setStreamName('rgb')

        #Properties and configuration

        #Stereo cameras
        self.monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        #Color camera
        self.colorCam.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        #Depth configuration for the disparity map

        self.depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        self.depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        self.depth.setLeftRightCheck(False)
        self.depth.setExtendedDisparity(False)
        self.depth.setSubpixel(False)

        config = self.depth.initialConfig.get()
        config.postProcessing.speckleFilter.enable = False
        config.postProcessing.speckleFilter.speckleRange = 50
        config.postProcessing.temporalFilter.enable = True
        config.postProcessing.spatialFilter.enable = True
        config.postProcessing.spatialFilter.holeFillingRadius = 2
        config.postProcessing.spatialFilter.numIterations = 1
        config.postProcessing.thresholdFilter.minRange = 400
        config.postProcessing.thresholdFilter.maxRange = 15000
        config.postProcessing.decimationFilter.decimationFactor = 1
        self.depth.initialConfig.set(config)

        #Linking 
        self.monoLeft.out.link(self.depth.left)
        self.monoRight.out.link(self.depth.right)
        self.depth.disparity.link(self.xout.input)
        self.colorCam.video.link(self.xoutRgb.input)

        #Ros2 publishers
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.disparity_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)

        #Bridge of OpenCV
        self.bridge = CvBridge()
        self.rgb_info = CameraInfo()
        self.get_logger().info("Initiate Oak-D-Lite ros2 node")
        self.frame_grabber()

    #Device connection and pipeline starts
    def frame_grabber(self):
        try:
            with dai.Device(self.pipeline) as device:

                # Output queues 
                qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                qDepth = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

                #Calibration
                calibData = device.readCalibration()
                intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT)
                self.get_logger().info(f'Right mono camera focal length in pixels: {intrinsics[0][0]}') #Camera calibration of focal length
    
                    
                while True:
                    
                    #Quues management
                    inRgb = qRgb.tryGet()
                    inDisparity = qDepth.tryGet()

                    now = self.get_clock().now().to_msg()

                    #Images publication

                    if inDisparity is not None:
                        
                        disp_frame = inDisparity.getCvFrame()
                        disp_frame_ros = self.bridge.cv2_to_imgmsg(disp_frame,'mono8')
                        disp_frame_ros.header.stamp = now
                        self.disparity_pub.publish(disp_frame_ros)
                        
                    if inRgb is not None:
                        rgb_frame = inRgb.getCvFrame()
                        rgb_frame_ros = self.bridge.cv2_to_imgmsg(rgb_frame, 'bgr8')
                        self.rgb_info.header.stamp = now
                        rgb_frame_ros.header.stamp = now
                        self.rgb_pub.publish(rgb_frame_ros)
                        self.rgb_info_pub.publish(self.rgb_info)

                        time.sleep(0.1)  
        except Exception as e:
            print("Error inesperado:", e)    
            sys.exit(1)
                
            
def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    stereo_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
