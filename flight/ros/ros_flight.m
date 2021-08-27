function log = ros_flight(traj,mode)

switch mode
    case 'gazebo'
        droneID = 'drone1';
        coreADD = 'relay.local';
%         coreADD = 'ASGARD.local';
%         coreADD = 'FOLKVANGR.local';
    case 'actual'
        droneID = 'drone2';
        %         droneID = 'drone7';
        coreADD = 'relay.local';
end

% Initialize ROS Matlab Node
try
    rosnode list
catch
    rosinit(coreADD)
end

%% Trajectory (single send)

% Initialize ROS Parameters
node     = ros.Node('/matlab_node');
pose_sub = ros.Subscriber(node,[droneID '/mavros/local_position/pose']);
vel_sub  = ros.Subscriber(node,[droneID '/mavros/local_position/velocity_local']);
th_sub   = ros.Subscriber(node,[droneID '/mavros/target_actuator_control']);
br_sub   = ros.Subscriber(node,[droneID '/mavros/setpoint_raw/target_attitude']);
volt_sub = ros.Subscriber(node,[droneID '/mavros/battery']);

x0_pub = ros.Publisher(node,[droneID '/setpoint/position'],'geometry_msgs/PoseStamped');
pause(1);

% Send the Drone to Initial Position
send2init(x0_pub,pose_sub,traj.x_bar(:,1));

% Send Trajectory for Execution
traj_client = ros.ServiceClient(node,[droneID '/setpoint/TrajTransfer'],"Timeout",3);

req = rosmessage(traj_client);
req.Hz = traj.hz;
req.N = size(traj.x_br,2);

req.UArr = traj.u_br(:);
req.LArr = traj.L_br(:);

req.XArr = traj.x_br(:);

call(traj_client,req,'Timeout',3);
% print(["Actual: ",num2str(cs_act)," ROS: ",num2str(cs_ros)]);

% Log
log = ros_logger(pose_sub,vel_sub,th_sub,br_sub,volt_sub,traj.t_fmu(1,end));

%% Hover Test
% pause(1);
% log = ros_logger(pose_sub,vel_sub,th_sub,br_sub,volt_sub,120);

