import numpy as np


def getTransform(pos, euler_ang):
    [x,y,z]          = pos
    [roll, pit, yaw] = np.array(euler_ang)
    
    Rx = np.array([[1,            0,              0,             0],
                   [0,            np.cos(roll),   -np.sin(roll), 0],
                   [0,            np.sin(roll),   np.cos(roll),  0],
                   [0,            0,              0,             1]])
    
    Ry = np.array([[np.cos(yaw),  0,              np.sin(yaw),   0],
                   [0,            1,              0,             0],
                   [-np.sin(yaw), 0,              np.cos(yaw),   0],
                   [0,            0,              0,             1]])
    
    
    Rz = np.array([[np.cos(pit), -np.sin(pit),   0,             0],
                   [np.sin(pit),  np.cos(pit),   0,             0],
                   [0,            0,             1,             0],
                   [0,            0,             0,             1]])
    
    T_pos = np.array([[1, 0, 0, x],
                      [0, 1, 0, y],
                      [0, 0, 1, z],
                      [0, 0, 0, 1]])
    
    Rxyz = np.dot(Rz,Ry).dot(Rx)
    
    return T_pos.dot(Rxyz)


def getJointTransforms(robot_urdf, leg, angles):
    tf_joints = {}

    for joint in robot_urdf.joints:
        if leg in joint.name:
            pos       = np.array(joint.origin.xyz)
            euler_ang = np.zeros(3)
            
            
            if 'hip1' in joint.name:
                euler_ang[0] = angles[0]
                tf_joints['hip1'] = getTransform(pos, euler_ang)
            
            if 'hip2' in joint.name:
                euler_ang[2] = angles[1]
                tf_joints['hip2'] = getTransform(pos, euler_ang)
                
            if 'knee' in joint.name:
                euler_ang[2] = angles[2]                
                tf_joints['knee'] = getTransform(pos, euler_ang)
            if 'feet' in joint.name:
                
                tf_joints['feet'] = getTransform(pos, euler_ang)
        
    return tf_joints


def getJointPositions(robot_urdf, leg, angles):
    tf_joints = getJointTransforms(robot_urdf, leg, angles)
        
    initial_pose = np.array([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
    
    
    pose_hip   = initial_pose.dot(tf_joints['hip1'])
    pose_femur = pose_hip.dot(tf_joints['hip2'])
    pose_tibia = pose_femur.dot(tf_joints['knee'])
    pose_feet  = pose_tibia.dot(tf_joints['feet'])
    

    pos_hip    = np.array([pose_hip[0, 3],   pose_hip[1, 3],   pose_hip[2, 3]])
    pos_femur  = np.array([pose_femur[0, 3], pose_femur[1, 3], pose_femur[2, 3]])
    pos_tibia  = np.array([pose_tibia[0, 3], pose_tibia[1, 3], pose_tibia[2, 3]])
    pos_feet   = np.array([pose_feet[0, 3],  pose_feet[1, 3],  pose_feet[2, 3]])
    
    return np.array([pos_hip, pos_femur, pos_tibia, pos_feet])