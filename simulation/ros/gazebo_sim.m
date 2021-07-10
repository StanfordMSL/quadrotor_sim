function log = gazebo_sim(traj,mode)

% Initialize ROS Matlab Node
% rosinit('ASGARD.local')

% Initialize ROS Topic
node = ros.Node('/matlab_node');

pose_sub = ros.Subscriber(node,'drone1/mavros/local_position/pose');
pause(1);

switch mode
    case 'body_rate'
        traj_client = ros.ServiceClient(node,'/drone1/setpoint/TrajTransfer',"Timeout",3);

        req = rosmessage(traj_client);
        req.Hz = traj.hz;
        req.N = size(traj.x,2);
        
        req.UArr = traj.u_br(:);
        req.LArr = traj.L(:);
        req.XArr = traj.x(:);
        
        call(traj_client,req,'Timeout',3);

        % print(["Actual: ",num2str(cs_act)," ROS: ",num2str(cs_ros)]);
end

t_g = zeros(1,5000);
x_g = zeros(3,5000);
curr_time = 0;
k = 1;
tic
while (curr_time <= traj.t_fmu(2))
    curr_time = toc;    
    t_g(1,k) = curr_time;
    
    pose = pose_sub.LatestMessage;

    x_g(1,k) = pose.Pose.Position.X;
    x_g(2,k) = pose.Pose.Position.Y;
    x_g(3,k) = pose.Pose.Position.Z;

    k = k + 1;

    pause(0.1);
end

log.t_g = t_g(1,1:k);
log.x_g = x_g(:,1:k);