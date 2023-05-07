#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from dz2.srv import read_command, read_commandResponse

speed_step  = 0.3
speed_const = 0.3
angle_step = np.pi/8
key_bindings = {
    'w': [speed_step, 0.0],
    'x': [-speed_step, 0.0],
    'd': [0.0, angle_step],
    'a': [0.0, -angle_step],
    's': [0.0, 0.0]
}

def manual_mode(msg, req, pub_cmd):
    rospy.loginfo('Manual mode is ON.')
    msg.linear.x  = 0
    msg.linear.y  = 0
    msg.linear.z  = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    
    # Choosing the direction of our movement.
    key = rospy.get_param('key', 'x')
    if key in key_bindings:
        linear_speed, angular_speed = key_bindings[key]
    msg.linear.x  = linear_speed
    msg.angular.z = angular_speed

    pub_cmd.publish(msg)

def auto_mode(msg, req, pub_cmd):
    # Getting the current position of the robot.
    global x, y, theta
    rospy.loginfo('Auto mode is ON.')

    # Calculating distance between target and current position.
    delta_x = req.x_target - x
    delta_y = req.y_target - y
    rho = np.sqrt((delta_x)**2 + (delta_y)**2)
    rospy.loginfo(f'Current distance {rho}')
    
    # Calculating needed movement to get to target.
    while (rho >= 0.1) or (abs(req.theta_target - theta) >= np.pi/16):
        alpha   = - theta + math.atan2(delta_y, delta_x)
        beta    = - theta - alpha + req.theta_target
        k_rho   = 0.2
        k_alpha = 0.5
        k_beta  = -0.2
        
        # Target is in front of the robot.
        if (alpha < np.pi/2) and (alpha > -np.pi/2):
            v = k_rho*rho
            w = k_alpha*alpha + k_beta*beta
        # Target is behind the robot.
        else:
            v     = -k_rho*rho
            alpha = alpha - np.pi*np.sign(alpha)
            beta  = beta  - np.pi*np.sign(beta)
            w     = k_alpha*alpha + k_beta*beta
        
        # Calculating distance for the next iteration.
        delta_x = req.x_target - x
        delta_y = req.y_target - y
        rho = np.sqrt((delta_x)**2 + (delta_y)**2)

        msg.linear.x = v
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = w

        pub_cmd.publish(msg)
    
    # Stopping robot when we reach the target.
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    pub_cmd.publish(msg)

def auto_const_mode(msg, req, pub_cmd):
    rospy.loginfo('Auto mode with const velocity is ON.')

    # Calculating distance between target and current position.
    delta_x = req.x_target-x
    delta_y = req.y_target-y
    rho = np.sqrt((delta_x)**2+(delta_y)**2)
    
    # Calculating needed movement to get to target.
    while (rho >= 0.1) or (abs(req.theta_target-theta) >= np.pi/16):
        alpha   = - theta + math.atan2(delta_y,delta_x)
        beta    = - theta - alpha + req.theta_target
        k_rho   = 0.2
        k_alpha = 0.5
        k_beta  = -0.2 
        
        # Target is in front of the robot.
        if (alpha<np.pi/2) and (alpha>-np.pi/2):
            v = k_rho*rho
            w = k_alpha*alpha + k_beta*beta
        # Target is behind the robot.
        else:
            v = -k_rho*rho
            alpha = alpha - np.pi*np.sign(alpha)
            beta  = beta - np.pi*np.sign(beta)
            w     = k_alpha*alpha + k_beta*beta
        
        # Calculating distance for the next iteration.
        delta_x = req.x_target-x
        delta_y = req.y_target-y
        rho = np.sqrt((delta_x)**2+(delta_y)**2)
        
        # While we are far enough from target, we are moving at constant speed.
        if(rho > 0.25):
            scaller = abs(v/w)                     # We use it so that we get w in rad/s units instead of m/s.
            v = speed_const*np.sign(v)
            w = speed_const/scaller*np.sign(w)

        msg.linear.x = v
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = w

        pub_cmd.publish(msg)
    
    # Stopping robot when we reach the target.
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0


def response_callback(req):

    msg = Twist()
    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	
    # Manual mode
    if(req.mode == 'M'):
        manual_mode(msg, req, pub_cmd)
			
    # Auto mode
    if(req.mode == 'A'):
        auto_mode(msg, req, pub_cmd)
        
    # Auto mode with const velocity brzinom		
    if(req.mode=='AC'):
        auto_const_mode(msg, req, pub_cmd)
	
    return read_commandResponse(True)

def callback(data):
    global x, y, theta
    x_ort = data.pose.pose.orientation.x
    y_ort = data.pose.pose.orientation.y
    z_ort = data.pose.pose.orientation.z
    rot_ort = data.pose.pose.orientation.w
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    _, _, theta = euler_from_quaternion([x_ort, y_ort, z_ort, rot_ort])

def listener():
    rospy.init_node('robot_controller')
    sub_odom = rospy.Subscriber('/odom', Odometry, callback)
    srv = rospy.Service('read_command', read_command, response_callback)
    rospy.loginfo('System is ready.')
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()

    except ROSInterruptException:
        pass    