#!/usr/bin/env python

#####################################################
##          RViz. Pose Array Publisher             ##
#####################################################
# Software License Agreement (BSD License)          #
# Author: Adam Buynak                               #
#####################################################

## IMPORTS ##
import rospy
import numpy as np
import geometry_msgs.msg

# Custom Scripts
from process_path import prepare_path_transforms_list, prepare_path_tf_ready

## SUPPORT FUNCTIONS ##
def rosmsg_geoPose(pose):
    ## Recieves dictionary and list formats and returns a tf2.geom.pose message

    # Get Current Orientation in Quanternion Format
    # http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html
    #q_poseCurrent = self.move_group.get_current_pose().pose.orientation
    #print(q_poseCurrent)

    # Using Quaternion's for Angle
    # Conversion from Euler(rotx,roty,rotz) to Quaternion(x,y,z,w)
    # Euler Units: RADIANS
    # http://docs.ros.org/en/melodic/api/tf/html/python/transformations.html
    # http://wiki.ros.org/tf2/Tutorials/Quaternions
    # http://docs.ros.org/en/api/geometry_msgs/html/msg/Quaternion.html

    if isinstance(pose,list):
        # Assume already in Quanternion angles in format xyzw
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.x = pose[3]
        pose_goal.orientation.y = pose[4]
        pose_goal.orientation.z = pose[5]
        pose_goal.orientation.w = pose[6]
    else:
        print("---> Incorrect Input Format")

    return pose_goal


## NODES ##
def node_cameraPoseArray(inputArray):
    """ Publish and Latch a Pose Array to a rostopic. """
    
    # Imports
    import rospy
    from geometry_msgs.msg import PoseArray

    # Config node
    pub = rospy.Publisher('cameraPoseArray', PoseArray, queue_size=10) #TODO: couldn't get latch=True to work. Looping instead
    rospy.init_node('cameraPoseArray', anonymous=False)
    rate = rospy.Rate(1) # 10hz

    message = geometry_msgs.msg.PoseArray()
    message.header.frame_id = 'base_link'
    message.poses = inputArray
    
    # Publish node
    while not rospy.is_shutdown():
        #rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()




## MAIN CODE ##
def main():



    # Process Path into Flat Vector Plane
    path_as_xyz = prepare_path_transforms_list('tiny_path_soln.csv',scaling_factor=0.05)

    # Convert Cartesian Points to Transformation List
    path_as_tf_matrices = prepare_path_tf_ready(path_as_xyz)

    # Create Maze Origin POSE
    import numpy as np
    from transformations import transformations
    tf = transformations()

    #body_rot = np.matrix('0 -1 0; 1 0 0; 0 0 1')   # rot(z,90deg)
    body_rot = np.matrix('0 -0.7071 0.7071; 1 0 0; 0 0.7071 0.7071')  # rot(z,90deg) rot(x,45deg)
    body_transl = np.matrix('0.3; 0.2; 0')
    body_frame = tf.generateTransMatrix(body_rot, body_transl)
    #print(body_frame)

    path_via_fixed_frame = tf.convertPath2FixedFrame(path_as_tf_matrices, body_frame)

    # Convert Path of Transforms to Robot Poses
    new_path_poses = tf.convertPath2RobotPose(path_via_fixed_frame)
    print(new_path_poses)

    # Generate PoseArray for ROS Node Publisher
    pose_geom = []
    for i in new_path_poses:
        pose_geom.append(rosmsg_geoPose(i))


    #print(pose_geom)
    #print(pose_geom[0])  #example of single, first pose vector


    # Try launching ros node
    try:
        node_cameraPoseArray(pose_geom)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
