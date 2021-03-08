#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import sys
import rospkg

global_path = Path()
global_path_pub = rospy.Publisher('global_path', Path,queue_size=1)
loaded = 0

def listener(arg):
    global file_path,global_path,loaded
    rospy.init_node('global_path_publisher', anonymous=True)
    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        global_path.header.stamp = rospy.Time.now()
        global_path.header.frame_id = "/map"


        if loaded == 0:

            #?
            rospack = rospkg.RosPack()
            rospack.list()

            # global_path stamp frame id set
            global_path.header.stamp = rospy.Time.now()
            global_path.header.frame_id = "/map"

            #save file's dir
            pkgpath = rospack.get_path('global_path_planner')

            #open file and read
            file = open(pkgpath + "/path_data/"+ str(arg) + ".txt",'r')
            line = file.readline().strip()


            #read x,y data of arg
            while(line):
               field = line.split()
               x = float(field[0])
               y = float(field[1])
               temp_pose = PoseStamped()
               temp_pose.header.stamp = rospy.Time.now()
               temp_pose.header.frame_id = "/map"
               if str(arg)=="2020_final_path0920":
                x -=302533.174487
                y -=4124215.34631
               
               temp_pose.pose.position.x = x
               temp_pose.pose.position.y = y
               global_path.poses.append(temp_pose)
               line = file.readline().strip()

               #just load one time
               loaded = loaded + 1

        #global path publish
        global_path_pub.publish(global_path)
        #print("pub!")
        rate.sleep()



if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
                print("usage: global_path_publisher.py {semi_path or final_path}")
        else:
            listener(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
