function nominal_plot(traj,obj,step,view_point,plot_type)

x_bar  = traj.x_bar;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define plot window and clear previous stuff
figure(3)
clf
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate flight room map
pnts_gate_rdr = [obj.pnts_gate obj.pnts_gate(:,1)];  % render points need to terminate at start
gate_h = plot3(pnts_gate_rdr(1,:)',pnts_gate_rdr(2,:)',pnts_gate_rdr(3,:)');
gate_h.LineWidth = 3;
xlim(obj.x_lim);
ylim(obj.y_lim);
zlim(obj.z_lim);
grid on
hold on

% Plot the Waypoints
for k = 1:size(obj.wp_arr,2)
    [x_arrow, y_arrow, z_arrow] = frame_builder(obj.wp_arr(:,k));
    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
        
    h_wp = plot3(x,y,z,'linewidth',2);
    
    % Set the Correct Colors
    h_wp(1).Color = [1 0 0];
    h_wp(2).Color = [0 1 0];
    h_wp(3).Color = [0 0 1];
end

switch plot_type
    case 'animate'        
        disp(['[nominal_plot]: Plotting at ',num2str(step),' step intervals of the FMU.']);
        for k = 1:step:size(x_bar,2)
            [x_arrow, y_arrow, z_arrow] = frame_builder(x_bar(:,k));
            x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
            y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
            z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

            h_fr = plot3(x,y,z,'linewidth',2);

            % Set the Correct Colors
            h_fr(1).Color = [1 0 0];
            h_fr(2).Color = [0 1 0];
            h_fr(3).Color = [0 0 1];
        end
    case 'static'
        [x_arrow, y_arrow, z_arrow] = frame_builder(x_bar(:,end));
        x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
        y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
        z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

        h_fr = plot3(x,y,z,'linewidth',2);
        
        h_fr(1).Color = [1 0 0];
        h_fr(2).Color = [0 1 0];
        h_fr(3).Color = [0 0 1];
end

% Plot the full trajectory
plot3(x_bar(1,:),x_bar(2,:),x_bar(3,:),'--k','linewidth',1);

% Set Aspect Ratio
daspect([1 1 1])

% Set View Point
switch view_point
    case 'persp'
        view(320,20);
%         zoom(1.8)
    case 'back'
        view(-90,0);    
    case 'side'
        view(0,0);
end


end

