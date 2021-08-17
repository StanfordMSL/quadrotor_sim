function point2traj(mode)

% Initialize ROS Matlab Node
try
    rosnode list
catch
    rosinit('relay.local')
end

% Initialize ROS Parameters
node = ros.Node('/matlab_node');
pose_pub = ros.Publisher(node,'drone7/setpoint/position','geometry_msgs/PoseStamped');
pose_sub = ros.Subscriber(node,'drone7/mavros/local_position/pose');
th_sub = ros.Subscriber(node,'drone7/mavros/target_actuator_control');
br_sub = ros.Subscriber(node,'drone7/mavros/setpoint_raw/target_attitude');
pause(1);

% Target Point
pose = pose_sub.LatestMessage;
point0 = [pose.Pose.Position.X ; pose.Pose.Position.Y; pose.Pose.Position.Z];

switch mode
    case 'climb'
        offset = [0 ; 0 ; 1];
    case 'drop'
        offset = [0 ; 0 ; 0.5];
    case 'short_line'
        offset = [1 ; 0 ; 0];
    case 'medium_line'
        offset = [2 ; 0 ; 0];
    case 'long_line'
        offset = [3 ; 0 ; 0];
    case 'slant_line'
        offset = [2 ; 1 ; 0];
end
point = point0 + offset;

% Send to Point to Generate Data
[T,X,U] = send2point(pose_pub,pose_sub,th_sub,br_sub,point);
D = [T ; X ; U];

% Save to csv
t_now = datetime;
t_now.Format = 'HHmmss';
save_folder = string(t_now);

mkdir('/home/lowjunen/StanfordMSL/quadrotor_sim/misc/sysID',save_folder);
address = strcat('/home/lowjunen/StanfordMSL/quadrotor_sim/misc/sysID','/',save_folder,'/',mode,'.csv');
csvwrite(address,D);

disp(['[point2traj]: Trajectory Saved To ' address]);
