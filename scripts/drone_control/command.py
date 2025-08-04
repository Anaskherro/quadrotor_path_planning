import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
from rdp import rdp
from iq_gnc.py_gnc_functions import * # type: ignore
from iq_gnc.PrintColours import * # type: ignore
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def main():

    # target course
    wpt_x = []
    wpt_y = []

    with open('path2.txt', 'r') as f:
        lines = f.readlines()
    
    for line in lines:
        ln = line.split(',')
        wpt_x.append(float(ln[0]))
        wpt_y.append(float(ln[1]))


    
    
    path_array = [[wpt_x[i], wpt_y[i]] for i in range(len(wpt_x))]
    path_array = np.array(path_array)
    path = rdp(path_array, epsilon=0.3)
    traj_x = list(path[:,0])
    traj_y = list(path[:,1])




    rospy.init_node("drone_controller", anonymous=True)

    # Create an object for the API.
    drone = gnc_api() # type: ignore
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3)
    # Specify control loop rate. We recommend a low frequency to not overload the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)
    # Init goals
    goals = []
    for x, y in zip(traj_x, traj_y):
        goals.append([x, -y, 3, 0])

    i=0
    # Create a publisher for goals as a Path
   
    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3]) #yaw control
        
        # Publish the goals as a Path message
        
        path_pub = rospy.Publisher('waypoints_path', Path, queue_size=10)
 
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'  # Adjust frame id according to your needs

        for goal in goals:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = goal[0]
            pose_stamped.pose.position.y = goal[1]
            pose_stamped.pose.position.z = goal[2]
            path.poses.append(pose_stamped)

        path_pub.publish(path)
        rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
    # Land after all waypoints are reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached, landing now." + CEND) # type: ignore


if __name__ == "__main__":
    main()
