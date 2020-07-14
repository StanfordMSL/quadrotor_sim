function nominal_plot(wp,nom,view_point,hz)

p_gate = wp.p_gate;

t_bar = nom.t_bar;
x_bar = nom.x_bar;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define plot window and clear previous stuff
figure(3)
clf
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate flight room map
gate_h = plot3(p_gate(1,:)',p_gate(2,:)',p_gate(3,:)');
gate_h.LineWidth = 3;
xlim(wp.x_lim);
ylim(wp.y_lim);
zlim(wp.z_lim);
grid on
hold on

% Plot the Waypoints
for k = 1:size(wp.x,2)
    [x_arrow, y_arrow, z_arrow] = frame_builder(wp.x(:,k));
    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
        
    h_wp = plot3(x,y,z,'linewidth',2);
    
    % Set the Correct Colors
    h_wp(1).Color = [1 0 0];
    h_wp(2).Color = [0 1 0];
    h_wp(3).Color = [0 0 1];
end

t_now = 0;
tol = 1e-3;
for k = 1:size(x_bar,2)
    if (abs(t_bar(k)-t_now) < tol)
        [x_arrow, y_arrow, z_arrow] = frame_builder(x_bar(:,k));
        x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
        y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
        z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
        
        h_fr = plot3(x,y,z,'linewidth',2);
    
        % Set the Correct Colors
        h_fr(1).Color = [1 0 0];
        h_fr(2).Color = [0 1 0];
        h_fr(3).Color = [0 0 1];
        t_now = t_now + 1/hz;
    end
end

% Set Camera Angle
daspect([1 1 1])
   
switch view_point
    case 'persp'
        view(320,20);
%         zoom(1.8)
    case 'back'
        view(-90,0);    
    case 'side'
        view(0,0);
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the full trajectory
plot3(x_bar(1,:),x_bar(2,:),x_bar(3,:),'--k','linewidth',1);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the Initial Frame  
% [x_arrow, y_arrow, z_arrow] = frame_builder(x_bar(:,1));
% x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
% y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
% z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
% 
% h_persp = plot3(x,y,z,'linewidth',3);
%     
% % Set the Correct Colors
% h_persp(1).Color = [1 0 0];
% h_persp(2).Color = [0 1 0];
% h_persp(3).Color = [0 0 1];
% 
% % Labels and Legend
% xlabel('x-axis');
% ylabel('y-axis');
% zlabel('z-axis');

end

