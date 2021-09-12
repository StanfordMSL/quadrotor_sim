function [node,subs,pubs,srvs,obj] = ros_start(droneID,gateID)

try
    rosnode list
catch
    rosinit('relay.local')
end
pause(0.1);

node = ros.Node('/matlab_node');
pause(0.1);

subs.gate = ros.Subscriber(node,['vrpn_client_node/' gateID '/pose']);
subs.pose = ros.Subscriber(node,[droneID '/mavros/vision_pose/pose']);
subs.vel  = ros.Subscriber(node,[droneID '/mavros/local_position/velocity_local']);
subs.th   = ros.Subscriber(node,[droneID '/mavros/target_actuator_control']);
subs.br   = ros.Subscriber(node,[droneID '/mavros/setpoint_raw/target_attitude']);

pubs.x0   = ros.Publisher(node,[droneID '/setpoint/position'],'geometry_msgs/PoseStamped');
pubs.traj = ros.Publisher(node,[droneID '/setpoint/TrajUpdate'],'bridge_px4/TrajUpdate');

srvs.traj = ros.ServiceClient(node,[droneID '/setpoint/TrajTransfer'],"Timeout",3);

pause(0.1);

gt_add = 'scenarios/gates/';
gates = dir([gt_add '*.csv']);
obj.kf.gt  = zeros(8,1);
for k_wp = 1:length(gates)
  file = [gt_add gates(k_wp).name];
  obj.db(k_wp).gt_dim = readmatrix(file,'Range', [1 2]);
  
  if strcmp(gates(k_wp).name(1:end-4),gateID)
      obj.kf.gt(1) = k_wp;
  end
end

% Obj. Type
obj.type = 'race';

% Obj. Map
obj.map.act = [
    -8.1 8.1;       % Actual Map x-limits (length)
    -3.2 3.2;       % Actual  Map y-limits (width)
     0.0 3.0];      % Actual  Map z-limits (height)      
obj.map.lim  = [
    -7.5 7.5;       % Traj Map x-limits (length)
    -2.7 2.7;       % Traj Map y-limits (width)
     0.2 2.5];      % Traj Map z-limits (height)
 
end