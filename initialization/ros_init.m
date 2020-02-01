function [js_sub,pose_sub,twist_sub,m_cmd_pub] = ros_init()    
    
    m_cmd_pub = rospublisher("/sim_link/m_cmd","std_msgs/Float32MultiArray");
    
    js_sub    = rossubscriber("/sim_link/pos_cmd");
    pose_sub  = rossubscriber("/gquad/mavros/local_position/pose");
    twist_sub = rossubscriber("/gquad/mavros/local_position/velocity");
end