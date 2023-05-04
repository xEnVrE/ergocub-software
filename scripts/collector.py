import argparse
import numpy
import pickle
import time
import yarp
from map import JointsMap
from pathlib import Path


class Collector():

    def __init__(self, args):

        yarp.Network.init()

        database = {}
        if Path('./database.pickle').exists():
            with open('./database.pickle', 'rb') as f:
                database = pickle.load(f)

        # Prepare a property object
        prefix = 'collector'
        props = yarp.Property()
        props.put('device','remote_controlboard')
        props.put('local','/' + prefix + '/left_arm')
        props.put('remote','/ergocubSim/left_arm')

        # Initialize ports for reading object pose
        if args.has_object:
            pose_port = yarp.BufferedPortVector()
            pose_port.open('/' + prefix + '/object_' + args.object_name + '/pose:i')
            yarp.Network.connect('/object_' + args.object_name + '/model-mover/pose:o', '/' + prefix + '/object_' + args.object_name + '/pose:i')

        # Initialize joints map
        map = JointsMap()

        # Driver initialization
        arm_driver = yarp.PolyDriver(props)
        iencoders = arm_driver.viewIEncoders()

        # Read data
        n_encs = iencoders.getAxes()
        data_yarp = yarp.Vector(n_encs)
        wait = 0.0
        t0 = time.time()
        while (time.time() - t0) < 0.1:
            iencoders.getEncoders(data_yarp.data())

        data_mapped = {}
        for name in map.joints_ctl:
            value = data_yarp[map.joints_map[name]]
            data_mapped[name] = value

        database[args.cfg_name] = {}
        database[args.cfg_name]['joints'] = data_mapped
        database[args.cfg_name]['has_object'] = args.has_object
        database[args.cfg_name]['object_name'] = args.object_name

        if args.has_object:
            t0 = time.time()
            while (time.time() - t0) < 0.1:
                object_pose_yarp = pose_port.read()
            object_pose = numpy.array([object_pose_yarp[0], object_pose_yarp[1], object_pose_yarp[2], object_pose_yarp[3], object_pose_yarp[4], object_pose_yarp[5], object_pose_yarp[6]])
            database[args.cfg_name]['object_pose'] = object_pose
        print(database[args.cfg_name])
        with open('./database.pickle', 'wb') as f:
            pickle.dump(database, f)

        arm_driver.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg-name', dest = 'cfg_name', type = str, required = True)
    parser.add_argument('--has-object', dest = 'has_object', action = 'store_true')
    parser.add_argument('--object-name', dest = 'object_name', type = str, default = '', required = False)

    args = parser.parse_args()

    Collector(args)
