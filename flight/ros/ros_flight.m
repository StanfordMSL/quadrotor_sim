function log = ros_flight(traj,obj,mode,type)

%% Initialize MATLAB -> ROS node

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

try
    rosnode list
catch
    rosinit(coreADD)
end

node = ros.Node('/matlab_node');

%% Define the relevant ROS publishers and subscribers

subs.pose = ros.Subscriber(node,[droneID '/mavros/local_position/pose']);
subs.vel  = ros.Subscriber(node,[droneID '/mavros/local_position/velocity_local']);
subs.th   = ros.Subscriber(node,[droneID '/mavros/target_actuator_control']);
subs.br   = ros.Subscriber(node,[droneID '/mavros/setpoint_raw/target_attitude']);

x0_pub   = ros.Publisher(node,[droneID '/setpoint/position'],'geometry_msgs/PoseStamped');
traj_pub = ros.Publisher(node,[droneID '/setpoint/TrajUpdate'],'bridge_px4/TrajUpdate');

pause(1);

%% Send the Drone to Initial Position
send2init(x0_pub,subs.pose,traj.x_bar(:,1));

%% Send Initial Trajectory for Execution
traj_client = ros.ServiceClient(node,[droneID '/setpoint/TrajTransfer'],"Timeout",3);

req = rosmessage(traj_client);
req.Hz = traj.hz;
req.N = size(traj.x_br,2);

req.UArr = traj.u_br(:);
req.LArr = traj.L_br(:);

req.XArr = traj.x_br(:);

res = call(traj_client,req,'Timeout',3);

%% Online Updates

switch type
    case 'single'
        log = ros_logger(subs,traj.t_fmu(1,end));
    case 'iterate'
        log = ros_update(subs,traj_pub,traj,obj,res);
end

