clear
rosshutdown 
pause(1);

% l_x = 0.10;
% l_y = 0.12;
l_x = 0.0;
l_y = 0.0;
r_d_arr = [ l_x    0.0  -l_x    0.0;
            0.0  -l_y    0.00   l_y; 
            0.0    0.0   0.00   0.0 ];
n_p = size(r_d_arr,2);

delt_map = 0.01.*[
    -11.547 -11.547 23.094  -11.547;
     20.000 -20.000  0.000   20.000;
      0.000   0.000  0.000    0.000];
delt_text = ["Lower","Right","Left"];
bulk_map = [
     -1.000  1.000 
      0.000  0.000 ;
     -0.100 -0.100 ];
  
droneID = 'drone2';
coreADD = 'relay.local';
rosinit(coreADD)
node = ros.Node('/matlab_node');
pause(1.0);

quad = ros.Subscriber(node,'drone2/mavros/vision_pose/pose');
delt = ros.Subscriber(node,'vrpn_client_node/delt/pose');
bulk = ros.Subscriber(node,'vrpn_client_node/bulk/pose');
gate = ros.Subscriber(node,'vrpn_client_node/gate/pose');
slit = ros.Subscriber(node,'vrpn_client_node/slit/pose');
pause(1.0);

pose = quad.LatestMessage.Pose;
p_quad = [pose.Position.X ; pose.Position.Y ; pose.Position.Z];
        
obs = delt;
map = delt_map;
while (norm(p_quad) < 5)
    pose = obs.LatestMessage.Pose;
    p_obs = [pose.Position.X ; pose.Position.Y ; pose.Position.Z];
    q_obs = [pose.Orientation.W ; pose.Orientation.X ; pose.Orientation.Y ; pose.Orientation.Z];
    
    pose = quad.LatestMessage.Pose;
    p_quad = [pose.Position.X ; pose.Position.Y ; pose.Position.Z];
    q_quad = [pose.Orientation.W ; pose.Orientation.X ; pose.Orientation.Y ; pose.Orientation.Z];

    if (norm(p_quad-p_obs) < 0.6)
        output = [];
        for j = 1:(size(map,2)-1)
            p0 = p_obs;
            p1 = p_obs + quatrotate(q_obs',map(:,j)')';
            p2 = p_obs + quatrotate(q_obs',map(:,j+1)')';

            for k = 1:1
                x = p_quad + quatrotate(q_quad',r_d_arr(:,k)')';
                
                if plane_calc(x,p0,p1,p2) > 0
                    output = [output delt_text(1,j)];
                else
                   % Do nothing
                end
            end
        end
        disp("Within: ");
        disp(output);
    else
        disp("Not in boundary")
    end
end