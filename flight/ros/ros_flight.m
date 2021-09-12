function log = ros_flight(traj,subs,pubs,srvs)


%% Send the Drone to Initial Position
send2init(pubs.x0,subs.pose,traj.x_bar(:,1));

%% Send Initial Trajectory for Execution

req = rosmessage(srvs.traj);
req.Hz = traj.hz;
req.N = size(traj.x_br,2);

req.UArr = traj.u_br(:);
req.LArr = traj.L_br(:);

req.XArr = traj.x_br(:);

call(srvs.traj,req,'Timeout',3);

%% Logging

log = ros_logger(subs,traj.t_fmu(1,end));

end

