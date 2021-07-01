function log = gazebo_sim(traj,mode)

% Tuning Parameters
pub_hz = traj.hz;
N = size(traj.x,2);

% Initialize ROS Matlab Node
% rosinit('ASGARD.local')
node = ros.Node('/matlab_node');
r = ros.Rate(node,pub_hz);

% Initialize ROS Topics
drone = "/drone1";
pos_topic = drone + "/setpoint/position";
set_mode_topic = drone + "/mavros/set_mode";
arming_topic = drone + "/mavros/cmd/arming";

pos_sp_pub = ros.Publisher(node,pos_topic,'geometry_msgs/PoseStamped');
set_mode_client = ros.ServiceClient(node,set_mode_topic,'DataFormat','struct');
arming_client = rssvcclient(arming_topic);

% Arm the Drone
req = rosmessage(set_mode_client);
req.custom_mode = "POSCTL";
call(set_mode_client,req,'Timeout',3);

pause(1);

req = rosmessage(set_mode_client);
req.custom_mode = "OFFBOARD";
call(set_mode_client,req,'Timeout',3);

pause(1);

req = rosmessage(arming_client);
req.value = 1;
call(arming_client,req,'Timeout',3);

pause(5);

% Publish Topic
reset(r)
for k_ros = 1:N
    tic
        
    msg = rosmessage(pos_sp_pub);
    
    msg.Header.Stamp = rostime('now','system','DataFormat','struct');
    msg.Header.Seq = k_ros;
    msg.Header.FramId = "world";
    msg.Pose.Position.X = traj.x(1,k);
    msg.Pose.Position.Y = traj.x(2,k); 
    msg.Pose.Position.Z = traj.x(3,k);
    
    send(pos_sp_pub,msg);
    
    waitfor(r);
    toc
end

% Disarm the Drone

log = 0;

rosshutdown