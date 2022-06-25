import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge,CvBridgeError
import cv2 as cv
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

class LineFollowing(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.image_sub=self.create_subscription(Image,'/camera',self.image_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.cmd_vel_pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.create_timer(0.3,self.timer_callback)
        self.bridge=CvBridge()
        self.twist=Twist()

    
    def image_callback(self,data):
        #Convert the ros image to an opencv image
        try:
            cv_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        #Crop the image and keep only the region of interest (roi)
        height, width, channels = cv_image.shape
        descentre = 150
        rows_to_watch = 50
        crop_cv_image = cv_image[(height)//2+descentre:(height) //2+(descentre+rows_to_watch)] [1:width]
        
        #convert the image to hsv format since it is easy to handle colors in this format.
        hsv_cv_image=cv.cvtColor(crop_cv_image,cv.COLOR_BGR2HSV)

        #Make a yellow mask. Keep yellow(since line is yellow)  and make the rest black
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([50, 255, 255])
        mask_cv_image=cv.inRange(hsv_cv_image,lower_yellow,upper_yellow)
        res_cv_image=cv.bitwise_and(crop_cv_image, crop_cv_image, mask=mask_cv_image)

        #calculate the center of yellow color.
        m=cv.moments(mask_cv_image,False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        self.pcx,self.pcy=cx,cy
        #draw the circle. This circle indicates where the robot should move.
        cv.circle(res_cv_image,(int(cx),int(cy)),5,(0,255,0),-2)
            
        #cv.imshow('cv image',cv_image)
        cv.imshow('crop image',crop_cv_image)
        #cv.imshow('hsv image',hsv_cv_image)
        cv.imshow('mask image',mask_cv_image)
        cv.imshow('res image',res_cv_image)

        cv.waitKey(1)

        self.proportional_movement_controller(width,height,cx,cy)

    def proportional_movement_controller(self,width,height,cx,cy):
        x_deviation=cx-width//2
        self.twist.linear.x=0.05
        self.twist.angular.z= - x_deviation /600

    def timer_callback(self):
        self.cmd_vel_pub.publish(self.twist)
        self.get_logger().info('publishing '+str(self.twist))


def main(args=None):
    rclpy.init(args=args)
    tmp=LineFollowing()
    rclpy.spin(tmp)
    tmp.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
