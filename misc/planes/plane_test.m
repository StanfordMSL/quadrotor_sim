clear

l_x = 0.10;
l_y = 0.12;
l_z = 0.035;
r_d_arr = [-l_x   l_x   l_x  -l_x  -l_x   l_x   l_x  -l_x;
            l_y   l_y  -l_y  -l_y   l_y   l_y  -l_y  -l_y;
            l_z   l_z   l_z   l_z  -l_z  -l_z  -l_z  -l_z];
n_p = size(r_d_arr,2);

delt_map = 0.01.*[
    -12.8  20.8  -13.9  -12.8;
     20.2   0.6   -19.1  20.2;
     0.00  0.00   0.00   0.00];
gate_map = 0.01.*[
    -29.3  29.9  29.7 -28.7 -29.3;
     28.4  28.7 -28.3 -28.5  28.4;
     0.00  0.00  0.00  0.00  0.00];
slit_map = 0.01.*[
    -9.00  9.30   9.1 -8.50 -9.00;
     18.7  18.8 -18.6 -18.4  18.7;
     0.00  0.00  0.00  0.00  0.00];
  
delt_text = ["Lower","Left","Right"];
bulk_map = [
     -1.000  1.000 
      0.000  0.000 ;
     -0.100 -0.100 ];
  
% rosshutdown 
% pause(1);
% droneID = 'drone2';
% coreADD = 'relay.local';
% rosinit(coreADD)
node = ros.Node('/matlab_node');
pause(1.0);

quad = ros.Subscriber(node,'drone2/mavros/vision_pose/pose');
wand = ros.Subscriber(node,'vrpn_client_node/wand/pose');
delt = ros.Subscriber(node,'vrpn_client_node/delt/pose');
bulk = ros.Subscriber(node,'vrpn_client_node/bulk/pose');
gate = ros.Subscriber(node,'vrpn_client_node/gate/pose');
slit = ros.Subscriber(node,'vrpn_client_node/slit/pose');
pause(1.0);

obs = delt;
map = delt_map;
drone = quad;

pose = drone.LatestMessage.Pose;
p_drone = [pose.Position.X ; pose.Position.Y ; pose.Position.Z];
     
[X,Y] = meshgrid(-1:.25:1);

while (norm(p_drone) < 5)
    pose = obs.LatestMessage.Pose;
    p_obs = [pose.Position.X ; pose.Position.Y ; pose.Position.Z];
    q_obs = [pose.Orientation.W ; pose.Orientation.X ; pose.Orientation.Y ; pose.Orientation.Z];
    q_obs = quatconj(q_obs')';
    
    pose = drone.LatestMessage.Pose;
    p_drone = [pose.Position.X ; pose.Position.Y ; pose.Position.Z];
    q_drone = [pose.Orientation.W ; pose.Orientation.X ; pose.Orientation.Y ; pose.Orientation.Z];
    q_drone = quatconj(q_drone')';
    
    if (norm(p_drone-p_obs) < 0.6)
         status = zeros(4,8);
%         output = [];
        for j = 1:(size(map,2)-1)
            p0 = p_obs;
            p1 = p_obs + quatrotate(q_obs',map(:,j)')';
            p2 = p_obs + quatrotate(q_obs',map(:,j+1)')';

            p12 = p2-p1;
            p10 = p0-p1;
            t  = (p10'*p12)/(p12'*p12);
            pn = p1 + t*(p2-p1);
            n  = p0-pn;
            Z = (-(n(1)*(X-pn(1))+n(2)*(Y-pn(2)))./n(3))+pn(3);
         
            edge = [p1 p2];

            for k = 1:8
                x = p_drone +  + quatrotate(q_drone',r_d_arr(:,k)')';
                
                if plane_calc(x,p0,p1,p2) > 0
                    status(j,k) = 0;
                else
                    status(j,k) = 1;
                end
            end
            
            plot3(p0(1),p0(2),p0(3),'*');
            hold on
            mesh(X,Y,Z)
            plot3(edge(1,:),edge(2,:),edge(3,:));         
        end
        plot3(p_drone(1),p_drone(2),p_drone(3),'o');
%         view(-90,0);
        view(-90,90);
        xlim([-2 2]);
        ylim([-2 2]);
        zlim([-0.5 2]);
         
        hold off
         
        if sum(sum(status)) > 0
            disp("Collision");
        else
            disp("Safe");
        end
    else
        disp("Not in boundary")
    end
    pause(0.2);
end