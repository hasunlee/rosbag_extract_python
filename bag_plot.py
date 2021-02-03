from rosbag import Bag
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def data_collect(file_path):
    
    def read_msg(bag, topic, parser):
        
        # make list with same name of "topic" at "bag" file, extract data with "parser" function from msg.
        return np.array([
            parser(time, msg)
            for (topic_, msg, time) in bag
            if topic_ == topic
        ])
    bag=Bag(file_path)
    bag = [
        (topic, msg, time)
        for (topic, msg, time)
        in bag.read_messages()
    ]
    # bag is an iterable of (topic, msg, time)
    start_time=bag[0][2].to_sec()
    def start_time_zero(extract_data):
        # make start time data to zero
        buff=extract_data.T
        buff[0]=buff[0]-start_time
        return buff.T
    def quat2rpy(extract_data):
        # make quaternian to roll, pitch, yaw
        buff=np.zeros([extract_data.shape[0], 4])
        i=0
        for (t, w, x,y,z) in extract_data:
            buff[i,0]=t
            r = R.from_quat([w, x, y, z])
            buff[i,1:]=r.as_euler('zyx', degrees=True)
            print(buff[i,1:])
            i=i+1
        return buff

    position = read_msg(
            bag, '/mavros_1/odar/pose',
            parser=lambda t, msg: (t.to_sec(), msg.pose.position.x, msg.pose.position.y,  msg.pose.position.z)
            )
    
    des_position = read_msg(
            bag, '/mavros_1/odar/desired_pose',
            parser=lambda t, msg: (t.to_sec(), msg.pose.position.x, msg.pose.position.y,  msg.pose.position.z)
            )
    rotation = read_msg(
        bag, '/mavros_1/odar/pose',
        parser=lambda t, msg: (t.to_sec(), msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.position.y,  msg.pose.position.z)
        )
    des_rotation = read_msg(
        bag, '/mavros_1/odar/desired_pose',
        parser=lambda t, msg: (t.to_sec(), msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.position.y,  msg.pose.position.z)
        )
    position=start_time_zero(position)
    des_position= start_time_zero(des_position)
    rotation = quat2rpy(start_time_zero(rotation))
    des_rotation = quat2rpy(start_time_zero(des_rotation))

    return position, des_position, rotation, des_rotation

def plot(pose_data, des_pose_data, rot_data, des_rot_data):
    plt.figure(1)
    plt.subplot(311)
    plt.plot(
        pose_data[:,0], pose_data[:,1], 'r', des_pose_data[:,0], des_pose_data[:,1], 'g',  label='position_x'
    )
    plt.grid()
    plt.subplot(312)
    plt.plot(
        pose_data[:,0], pose_data[:,2], 'r', des_pose_data[:,0], des_pose_data[:,2], 'g',  label='position_y'
    )
    plt.grid()
    plt.subplot(313)
    plt.plot(
        pose_data[:,0], pose_data[:,3], 'r', des_pose_data[:,0], des_pose_data[:,3], 'g',  label='position_z'
    )
    plt.grid()
    plt.suptitle("position")

    plt.figure(2)
    plt.subplot(311)
    plt.plot(
        rot_data[:,0], rot_data[:,1], 'r', des_rot_data[:,0], des_rot_data[:,1], 'g', label='rotation_x'
    )
    plt.grid()
    plt.subplot(312)
    plt.plot(
        rot_data[:,0], rot_data[:,2], 'r', des_rot_data[:,0], des_rot_data[:,2], 'g', label='rotation_x'
    )
    plt.grid()
    plt.subplot(313)
    plt.plot(
        rot_data[:,0], rot_data[:,3], 'r', des_rot_data[:,0], des_rot_data[:,3], 'g', label='rotation_x'
    )
    plt.suptitle("rotation")
    plt.grid()
    plt.show()
    

bag_path =  './2020_12_29/Bags/12_29/1_0_degree.bag'
pose_data, des_pose_data, rot_data, des_rot_data = data_collect(bag_path)
plot(pose_data, des_pose_data, rot_data, des_rot_data)

