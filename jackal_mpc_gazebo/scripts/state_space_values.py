#!/usr/bin/env python3


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import csv

state_control_dict = {}



def write_to_csv():
    with open('data.csv' , 'w') as f:
        fields = ['x','y','phi','vx','vy','w']
        writer = csv.DictWriter(f,fieldnames=fields)
        writer.writeheader()

def odom_callback(odom):
    global x, y, yaw
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    orientation = odom.pose.pose.orientation
    quaternion = [orientation.x,orientation.y,orientation.z,orientation.w]
    [_,_,yaw] = euler_from_quaternion(quaternion)
    rospy.loginfo("X = "+str(x)+"Y = "+str(y)+"Phi = "+str(yaw))

    

def control_callback(input):
    vx = input.linear.x
    vy = input.linear.y
    thetaz = input.angular.z

    if vx !=0 or vy !=0 or thetaz !=0 :
        state_control_dict['x'] = x
        state_control_dict['y'] = y
        state_control_dict['phi'] = phi
        state_control_dict['vx'] = vx
        state_control_dict['vy'] = vy
        state_control_dict['w'] = thetaz
        with open('data.csv' , 'a') as f:
            fields = ['x','y','phi','vx','vy','w']
            writer = csv.DictWriter(f,fieldnames=fields)
            writer.writerow(state_control_dict)
        # df_state = pd.DataFrame([state_control_dict])
        # df = pd.concat([df,df_state],ignore_index=True)




""" def pos_callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    rospy.loginfo("Poistion received: x: "+str(x)+" y: "+str(y)) """

def get_state_space_values():
    #[x,y,phi,theta]

    rospy.init_node('state_space_sub',anonymous=True)
    #rospy.Subscriber("/base_link_pose",PoseWithCovarianceStamped,pos_callback)
    rospy.Subscriber("/odometry/filtered",Odometry,odom_callback)
    rospy.Subscriber("/jackal_velocity_controller/cmd_vel",Twist,control_callback)

    rospy.spin()


if __name__ == '__main__':
    write_to_csv()
    get_state_space_values()
