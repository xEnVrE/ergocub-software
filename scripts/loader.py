import argparse
import numpy
import pickle
import time
import yarp
from map import JointsMap
from pathlib import Path


class Demo():

    def __init__(self, args):

        prefix = 'demo'

        yarp.Network.init()

        if not Path('./database.pickle').exists():
            print('Error: cannot find database.pickle')
            exit(1)

        database = None
        with open('./database.pickle', 'rb') as f:
            database = pickle.load(f)

        # List of objects to be moved
        list_objects = ['011_banana', '006_mustard_bottle', '010_potted_meat_can', '003_cracker_box', '025_mug', '019_pitcher_base', '005_tomato_soup_can', '035_power_drill']

        # Port to send the new object poses to
        ports_pose_out = {}
        for name in list_objects:
            ports_pose_out[name] = yarp.Port()
            ports_pose_out[name].open('/' + prefix + '/object_' + name + '/model_mover/pose:o')
            yarp.Network.connect('/' + prefix + '/object_' + name + '/model_mover/pose:o', '/object_' + name + '/model-mover/pose:i/transform:i')

        # Helper
        def move_object_to(port, pose):

            pose_yarp = yarp.Vector(pose.shape[0])
            for i in range(pose.shape[0]):
                pose_yarp[i] = pose[i]

            port.write(pose_yarp)

        # Restore the object considered by this configuration
        if database[args.cfg_name]['has_object']:
            port = ports_pose_out[database[args.cfg_name]['object_name']]
            pose = database[args.cfg_name]['object_pose']
            t0 = time.time()
            while (time.time() - t0) < 0.1:
                move_object_to(port, pose)

        # Move to identity all the other objects
        for name in list_objects:
            if database[args.cfg_name]['has_object'] and \
               name == database[args.cfg_name]['object_name']:
                continue

            pose_zero = numpy.zeros(7)
            pose_zero[0] = 100.0
            pose_zero[1] = 100.0
            pose_zero[3] = 1.0
            port = ports_pose_out[name]
            t0 = time.time()
            while (time.time() - t0) < 0.1:
                move_object_to(port, pose_zero)

        map = JointsMap()

        # Joints desired velocities
        joints_ref_speed = {'thumb_oppose' : 50.0, 'thumb' : 50.0, 'index_add' : 50.0, 'index' : 50.0, 'middle' : 50.0, 'pinky' : 50.0, 'index_2_shifter' : 0.05, 'index_3_shifter' : 0.05, 'middle_1_shifter' : 0.05, 'middle_2_shifter' : 0.05, 'ring_1_shifter' : 0.05, 'ring_2_shifter' : 0.05, 'pinkie_1_shifter' : 0.05, 'pinkie_2_shifter' : 0.05, 'thumb_pivot' : 50.0, 'thumb_2_base_shifter' : 0.05, 'thumb_2_shifter' : 0.05, 'thumb_3_shifter' : 0.05}

        # Joints desired values for 'zero' configuration
        joints_des_zero = {'thumb_oppose' : 0.0, 'thumb' : 0.0, 'index_add' : 0.0, 'index' : 0.0, 'middle' : 0.0, 'pinky' : 0.0, 'index_2_shifter' : 0.0, 'index_3_shifter' : 0.0, 'middle_1_shifter' : 0.0, 'middle_2_shifter' : 0.0, 'ring_1_shifter' : 0.0, 'ring_2_shifter' : 0.0, 'pinkie_1_shifter' : 0.0, 'pinkie_2_shifter' : 0.0, 'thumb_pivot' : 0.0, 'thumb_2_base_shifter' : 0.0, 'thumb_2_shifter' : 0.0, 'thumb_3_shifter' : 0.0}

        joints_des_custom = {'thumb_oppose' : 0.0, 'thumb' : 0.0, 'index_add' : 0.0, 'index' : 0.0, 'middle' : 0.0, 'pinky' : 0.0, 'index_2_shifter' : 0.0, 'index_3_shifter' : 0.0, 'middle_1_shifter' : 0.0, 'middle_2_shifter' : 0.0, 'ring_1_shifter' : 0.0, 'ring_2_shifter' : 0.0, 'pinkie_1_shifter' : 0.0, 'pinkie_2_shifter' : 0.0, 'thumb_pivot' : 0.0, 'thumb_2_base_shifter' : 0.0, 'thumb_2_shifter' : 0.0, 'thumb_3_shifter' : 0.0}

        # Assign joints desired value
        joints_des = joints_des_zero
        if args.cfg_name == 'custom':
            joints_des = joints_des_custom
        elif args.cfg_name != 'zero':
            joints_des = database[args.cfg_name]['joints']

        # Prepare vector with joints indexes
        joints = yarp.VectorInt(len(map.joints_ctl))
        for i in range(len(map.joints_ctl)):
            name = map.joints_ctl[i]
            joints[i] = map.joints_map[name]

        # Prepare a property object
        props = yarp.Property()
        props.put('device','remote_controlboard')
        props.put('local','/' + prefix + '/left_arm')
        props.put('remote','/ergocubSim/left_arm')

        # Driver initialization
        arm_driver = yarp.PolyDriver(props)
        ipos = arm_driver.viewIPositionControl()
        control_mode = arm_driver.viewIControlMode()

        # Set control mode
        joints_mode = yarp.VectorInt(len(map.joints_ctl))
        for i in range(len(map.joints_ctl)):
            joints_mode[i] = yarp.VOCAB_CM_POSITION
        control_mode.setControlModes(len(map.joints_ctl), joints.data(), joints_mode.data())

        # Set reference velocity
        joints_vel = yarp.Vector(len(map.joints_ctl))
        for i in range(len(map.joints_ctl)):
            name = map.joints_ctl[i]
            joints_vel[i] = joints_ref_speed[name]
        ipos.setRefSpeeds(len(map.joints_ctl), joints.data(), joints_vel.data())

        # Helper
        def go_to(iface, joints_ctl, joints, joints_map, joints_des):
            values = yarp.Vector(len(joints_ctl))
            for i in range(len(joints_ctl)):
                name = joints_ctl[i]
                values[i] = joints_des[name]
            iface.positionMove(len(joints_ctl), joints.data(), values.data())

        # Set configuration
        go_to(ipos, map.joints_ctl, joints, map.joints_map, joints_des)

        # Close the driver
        arm_driver.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--list-cfgs', dest = 'list_cfgs', action = 'store_true')
    parser.add_argument('--cfg-name', dest = 'cfg_name', type = str, default = 'zero', required = False)
    parser.add_argument('--remove-cfg', dest = 'remove_cfg', action = 'store_true')
    parser.add_argument('--edit-cfg', dest = 'edit_cfg', action = 'store_true')
    parser.add_argument('--edit-joint-name', dest = 'edit_joint_name', type = str)
    parser.add_argument('--edit-joint-value', dest = 'edit_joint_value', type = float)

    args = parser.parse_args()

    if args.list_cfgs:
        if not Path('./database.pickle').exists():
            print('Error: cannot find database.pickle')
            exit(1)

        database = None
        with open('./database.pickle', 'rb') as f:
            database = pickle.load(f)

        print(list(database.keys()))
        exit(0)

    if args.remove_cfg:
        if not Path('./database.pickle').exists():
            print('Error: cannot find database.pickle')
            exit(1)

        database = None
        with open('./database.pickle', 'rb') as f:
            database = pickle.load(f)
        database.pop(args.cfg_name)
        with open('./database.pickle', 'wb') as f:
            pickle.dump(database, f)

        exit(0)

    if args.edit_cfg:
        if not Path('./database.pickle').exists():
            print('Error: cannot find database.pickle')
            exit(1)

        database = None
        with open('./database.pickle', 'rb') as f:
            database = pickle.load(f)
        database[args.cfg_name]['joints'][args.edit_joint_name] = args.edit_joint_value
        with open('./database.pickle', 'wb') as f:
            pickle.dump(database, f)

        exit(0)


    Demo(args)
