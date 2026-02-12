import os
import rospy
import rospkg

gait_struct = {}
gait_struct["walk"]            = 0
gait_struct["gait_period"]     = 0
gait_struct["prop_swing_time"] = 0

gait_struct["fl_phase_offset"] = 0
gait_struct["fr_phase_offset"] = 0
gait_struct["bl_phase_offset"] = 0
gait_struct["br_phase_offset"] = 0

gait_struct["max_xy_vel"] = 0
gait_struct["max_th_vel"] = 0

############# Feet and base settings ############
gait_struct["feet_offset_x"] = 0
gait_struct["feet_offset_y"] = 0
gait_struct["base_offset_x"] = 0
gait_struct["base_offset_y"] = 0
gait_struct["step_height"]   = 0
gait_struct["force_contact_threshold"] = 0

########### Predictive support polygon settings ###########
gait_struct["enable_predictive_support_polygon"] = 0
gait_struct["wfactor_stance_sigma1"] = 0
gait_struct["wfactor_stance_sigma2"] = 0
gait_struct["wfactor_swing_sigma1"]  = 0
gait_struct["wfactor_swing_sigma2"]  = 0
gait_struct["wfactor_offset"]        = 0

############### Force controller settings ####################
gait_struct["enable_force_controller"]          = 0
gait_struct["force_controller_k_base_position"] = 0
gait_struct["force_controller_k_base_velocity"] = 0

gait_struct["force_controller_k_base_angles"]      = 0
gait_struct["force_controller_k_base_angular_vel"] = 0

gait_struct["force_controller_dynamical_model_weight"] = 0
gait_struct["force_controller_regularization_weight"]  = 0
gait_struct["force_controller_prev_force_weight"]      = 0

gait_struct["force_controller_min_force"] = 0
gait_struct["force_controller_max_force"] = 0



def parseGait(gait_file):
    if '.gait' not in gait_file:
        gait_file += '.gait'
    rospack = rospkg.RosPack()
    GAITS_DIR = rospack.get_path('gait_control') + '/gaits/'
    GAIT_FILE = GAITS_DIR + gait_file

    f = open(GAIT_FILE, 'r')

    for line in f.readlines():
        if '#' not in line and line != '\n':
            key = line.split(' ')[0]
            value = float(line.split(' ')[-1])
            gait_struct[key] = value
    print(gait_struct)


parseGait('diagonal')