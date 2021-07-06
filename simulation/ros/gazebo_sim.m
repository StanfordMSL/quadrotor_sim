function log = gazebo_sim(traj,mode)

% Initialize ROS Matlab Node
% rosinit('ASGARD.local')

% Initialize ROS Topic
node = ros.Node('/matlab_node');

switch mode
    case 'body_rate'
        traj_client = ros.ServiceClient(node,'/drone1/setpoint/TrajTransfer',"Timeout",3);

        req = rosmessage(traj_client);
        req.Hz = traj.hz;
        req.N = size(traj.x,2);
        req.UArr = traj.u_br(:);
        req.LArr = traj.L(:);

        cs_act = sum(req.UArr) + sum(req.LArr);

        res = call(traj_client,req,'Timeout',3);
        cs_ros = res.Checksum;

        % print(["Actual: ",num2str(cs_act)," ROS: ",num2str(cs_ros)]);

        log = 0;
end