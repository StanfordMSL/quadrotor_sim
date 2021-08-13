function log = gazebo_sim(traj,mode)

% Initialize ROS Matlab Node
try
    rosnode list
catch
%     rosinit('ASGARD.local')
    rosinit('relay.local')
end

% droneID = 'drone1';
droneID = 'drone7';

% Initialize ROS Parameters
node = ros.Node('/matlab_node');
pose_sub = ros.Subscriber(node,[droneID '/mavros/local_position/pose']);
pose_init_pub = ros.Publisher(node,[droneID '/setpoint/position'],'geometry_msgs/PoseStamped');
pause(1);

% Send the Drone to Initial Position
send2init(pose_init_pub,traj.x_bar(:,1));
pause(3);

% Send Trajectory for Execution
switch mode
    case 'body_rate'
        traj_client = ros.ServiceClient(node,[droneID '/setpoint/TrajTransfer'],"Timeout",3);

        req = rosmessage(traj_client);
        req.Hz = traj.hz;
        req.N = size(traj.x_br,2);
        
        req.UArr = traj.u_br(:);
        req.LArr = traj.L_br(:);

        req.XArr = traj.x_br(:);
        
        call(traj_client,req,'Timeout',3);

        % print(["Actual: ",num2str(cs_act)," ROS: ",num2str(cs_ros)]);
end

log = ros_logger(pose_sub,traj.t_fmu(1,end));

