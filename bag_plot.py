from rosbag import Bag
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from argparse import ArgumentParser
import sys


def parse_arguments(args):
    parser = ArgumentParser()
    parser.add_argument('bag', help = 'bagfile to extract data',  type = str)
    parser.add_argument('link_total', help = 'how many links are there', type = int) 
    parser.add_argument('link_plot', help = 'link to plot', type = int)
    parser.add_argument('--position', help= 'plot position & desired position', action = 'store_true')
    parser.add_argument('--rotation', help= 'plot rotation & desired rotation', action = 'store_true')
    return parser.parse_args(args)

def data_collect(file_path, link_total):
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
            #print(buff[i,1:])
            i=i+1
        return buff

    position_list=[]
    des_position_list=[]
    rotation_list=[]
    des_rotation_list=[]

    for i in range(1,link_total+1):
        position = read_msg(
            bag, '/mavros_' + str(i) +'/odar/pose',
            parser=lambda t, msg: (t.to_sec(), msg.pose.position.x, msg.pose.position.y,  msg.pose.position.z)
            )
        position=start_time_zero(position)
        position_list.append(position)

        des_position = read_msg(
            bag, '/mavros_' + str(i) +'/odar/desired_pose',
            parser=lambda t, msg: (t.to_sec(), msg.pose.position.x, msg.pose.position.y,  msg.pose.position.z)
            )
        des_position = start_time_zero(des_position)
        des_position_list.append(des_position)

        rotation = read_msg(
            bag, '/mavros_' + str(i) +'/odar/pose',
            parser=lambda t, msg: (t.to_sec(), msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.position.y,  msg.pose.position.z)
            )
        rotation = quat2rpy(start_time_zero(rotation))
        rotation_list.append(rotation)

        des_rotation = read_msg(
            bag, '/mavros_' + str(i) +'/odar/desired_pose',
            parser=lambda t, msg: (t.to_sec(), msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.position.y,  msg.pose.position.z)
            )
        des_rotation = quat2rpy(start_time_zero(des_rotation))
        des_rotation_list.append(des_rotation)

    return position_list, des_position_list, rotation_list, des_rotation_list

def plot(pose_data, des_pose_data, rot_data, des_rot_data, pose_bool, rot_bool):
    if pose_bool:
        plt.figure()
        plt.plot(pose_data[:,0], pose_data[:,1],'r',  des_pose_data[:,0], des_pose_data[:,1],'r',  label='x')
        plt.plot(pose_data[:,0], pose_data[:,2], 'g',  des_pose_data[:,0], des_pose_data[:,2],'g',  label='y')
        plt.plot(pose_data[:,0], pose_data[:,3], 'b', des_pose_data[:,0], des_pose_data[:,3],'b',  label='z')
        plt.grid()
        plt.legend()
        plt.title("position")
    elif rot_bool:
        plt.figure()
        plt.plot(rot_data[:,0], rot_data[:,1], 'r', des_rot_data[:,0], des_rot_data[:,1], 'r', label='rotation_r')
        plt.plot(rot_data[:,0], rot_data[:,2], 'g', des_rot_data[:,0], des_rot_data[:,2], 'g', label='rotation_p')
        plt.plot(rot_data[:,0], rot_data[:,3], 'b', des_rot_data[:,0], des_rot_data[:,3], 'b', label='rotation_y')
        plt.grid()
        plt.legend()
        plt.title("rotation")
    plt.show()
    

def main(argv= sys.argv[1:]):
    args = parse_arguments(argv)
    list_pose, list_des_pose, list_rot, list_des_rot = data_collect(args.bag, args.link_total)
    plot(list_pose[args.link_plot-1], list_des_pose[args.link_plot-1], list_rot[args.link_plot-1], list_des_rot[args.link_plot-1], args.position, args.rotation)
    #print(list_rot[1])
    #print(list_pose[1])
if __name__ == '__main__':  # if code run directly at shell, __name__ has __main__ value.
    # if code import from other code, __name__ has "module name".
    main()


