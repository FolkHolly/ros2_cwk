import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Vector3
from nav2_msgs.action import NavigateToPose
from rclpy.exceptions import ROSInterruptException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from math import sin, cos, pi
import cv2
import numpy as np
import signal
import threading


class GoToPose(Node):
    def __init__(self):
        super().__init__('navigation_goal_action_client')
        # params for automatic exploration
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_coords = [[7., -11.5], [-9.5, 3.], [-8., -13.], [6., 4.]]
        self.current_goal = 0
        self.explore_mode = True
        
        # params for finding the colour
        self.blue_found = False
        self.sensitivity = 10
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        
        # params for moving to the colour
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)
        self.stop = False
        self.target = 500
        

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        x = self.goal_coords[self.current_goal][0]
        y = self.goal_coords[self.current_goal][1]
        print('new goal:', x, y)
        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(0.0 /2)
        goal_msg.pose.pose.orientation.w = cos(0.0 /2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        print('Goal sent')
        self.current_goal += 1


    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            print('Goal rejected')
            return

        print('Goal accepted')
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        print('Navigation goal reached')
        if self.current_goal < 3  and self.explore_mode:
            self.send_goal()


    def feedback_callback(self, feedback_msg):
        if self.explore_mode == False:
            # cancel route and move towards object
            print('object found, shifting navigation mode')
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_callback)
            
    def cancel_callback(self, cancel_msg):
        print(cancel_msg)
        
        
    def camera_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            print(e)
        
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        
        hsv_red0_lower = np.array([0, 100, 100])
        hsv_red0_upper = np.array([0 + self.sensitivity, 255, 255])
        hsv_red180_lower = np.array([180 - self.sensitivity, 100, 100])
        hsv_red180_upper = np.array([180, 255, 255])
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        red0_mask = cv2.inRange(hsv_image, hsv_red0_lower, hsv_red0_upper)
        red180_mask = cv2.inRange(hsv_image, hsv_red180_lower, hsv_red180_upper)  

        red_mask = cv2.bitwise_or(red0_mask, red180_mask)
        bg_mask = cv2.bitwise_or(blue_mask, green_mask)
        all_mask = cv2.bitwise_or(red_mask, bg_mask)
        
        blue_contours, blue_hierachy = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        all_countours, all_hierachy = cv2.findContours(all_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # draw bounding box around each contour:
        minArea = 30
        for contour in all_countours:
            if cv2.contourArea(contour) > minArea:
                x,y,w,h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x,y), (x+w, y+h), (255, 255, 255), 5)
        # show filtered image in window       
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)
        # Loop over the contours
        if len(blue_contours)>0:
            c = max(blue_contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if cv2.contourArea(c) > minArea:
                    self.blue_found = True
                else:
                    self.blue_found = False
                #Check if a flag has been set = colour object detected - follow the colour object
                if self.blue_found == True:
                    self.explore_mode = False
                    if cv2.contourArea(c) > 100000:
                        # Too close to object, need to move backwards
                        self.stop = True
                        
                    elif cv2.contourArea(c) < 100000:
                        # Too far away from object, need to move forwards
                        self.target = cx
             
        
                    
    def stop_moving(self):
        print('stopping')        
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0 

        self.publisher.publish(desired_velocity)
    
    def move_to_blue(self, target):
        print('moving to blue')
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2
        desired_velocity.angular.z = (500-target)*pi/(16*500)
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()
            
    def spin(self):
        print('lost blue - spin')
        desired_velocity = Twist()
        desired_velocity.angular.z = pi/2
        for _ in range(10):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

def main(args=None):
    def signal_handler(sig, frame):
        go_to_pose.stop_moving()
        rclpy.shutdown()

    rclpy.init(args=None)
    go_to_pose = GoToPose()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(go_to_pose,), daemon=True)
    thread.start()

    try:
        go_to_pose.send_goal()
        while rclpy.ok():
            if go_to_pose.stop == True:
                go_to_pose.stop_moving()
            elif go_to_pose.explore_mode == False:
                if go_to_pose.blue_found == True:
                    go_to_pose.move_to_blue(go_to_pose.target)
                else:
                    go_to_pose.spin()

    except ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
