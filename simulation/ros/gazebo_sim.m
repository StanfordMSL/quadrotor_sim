function log = gazebo_sim(traj,mode)

% Tuning Parameters
pub_hz = traj.hz;
N = size(traj.x,2);

% Initialize ROS Matlab Node
% rosinit('ASGARD.local')
node = ros.Node('/matlab_node');
r = ros.Rate(node,pub_hz);

% Initialize ROS Topic
% pos_topic = "/drone1/setpoint/position";
raw_topic = "/drone1/setpoint/attitude";

raw_sp_pub = ros.Publisher(node,raw_topic,'mavros_msgs/AttitudeTarget');

% Publish Topic
% reset(r)
msg = rosmessage(raw_sp_pub);

for k_ros = 1:(N-1) 
%     tic
    
    msg.Header.Stamp = rostime('now','system');
    msg.Header.Seq = k_ros;
    msg.Header.FrameId = "map";
    
%     msg.TypeMask = msg.IGNORETHRUST;
    msg.TypeMask = msg.IGNOREATTITUDE;
    
%     msg.Orientation.W = traj.x(7,k_ros);
%     msg.Orientation.X = traj.x(8,k_ros);
%     msg.Orientation.Y = traj.x(9,k_ros); 
%     msg.Orientation.Z = traj.x(10,k_ros);
    
    msg.BodyRate.X = traj.u(2,k_ros);
    msg.BodyRate.Y = traj.u(3,k_ros);
    msg.BodyRate.Z = traj.u(4,k_ros);
%     msg.BodyRate.X = 0.0;
%     msg.BodyRate.Y = 0.0;
%     msg.BodyRate.Z = 0.0;
    
%     msg.Thrust = 0.7;
    msg.Thrust = 0.13*traj.u(1,k_ros);
    
    send(raw_sp_pub,msg);
    
    waitfor(r);
%     toc
end

% Disarm the Drone

log = 0;

