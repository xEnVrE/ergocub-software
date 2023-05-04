class JointsMap():

    def __init__(self):

        # Create joint maps
        self.joints_map = {'thumb_oppose' : 0, 'thumb' : 1, 'index_add' : 2, 'index' : 3, 'middle' : 4, 'pinky' : 5, 'index_2_shifter' : 6, 'index_3_shifter' : 7, 'middle_1_shifter' : 8, 'middle_2_shifter' : 9, 'ring_1_shifter' : 10, 'ring_2_shifter' : 11, 'pinkie_1_shifter' : 12, 'pinkie_2_shifter' : 13, 'thumb_pivot' : 14, 'thumb_2_base_shifter' : 15, 'thumb_2_shifter' : 16, 'thumb_3_shifter' : 17}

        # Joints to be controlled
        self.joints_ctl = ['thumb_oppose', 'thumb', 'index_add', 'index', 'middle', 'pinky', 'thumb_pivot', 'thumb_2_base_shifter', 'index_2_shifter', 'index_3_shifter', 'middle_1_shifter', 'middle_2_shifter', 'ring_1_shifter', 'ring_2_shifter', 'pinkie_1_shifter', 'pinkie_2_shifter', 'thumb_pivot', 'thumb_2_base_shifter', 'thumb_2_shifter', 'thumb_3_shifter']
