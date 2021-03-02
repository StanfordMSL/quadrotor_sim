function problem_plot(map,obj,view_point)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define plot window and clear previous stuff
figure(1)
cla
set(gca,'ColorOrder','factory')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate flight room map
if any(ismember(fields(map),'p_gc')) == 1
    N_g = size(map.p_gc,3);
    
    for k = 1:N_g
        % Gate(s) Present. Render.    
        p_G1 = map.p_gc(:,1,k);
        p_G2 = map.p_gc(:,2,k);
        p_G4 = map.p_gc(:,4,k);

        r_12 = p_G2 - p_G1;
        r_14 = p_G4 - p_G1;
        n_G  = cross(r_14,r_12);
        p_gc_dir = map.p_gc(:,1,k)+ (0.3.*n_G./norm(n_G));
        
        gate = [map.p_gc(:,:,k) map.p_gc(:,1,k) p_gc_dir];  % render points need to terminate at start

        gate_h = plot3(gate(1,:)',gate(2,:)',gate(3,:)','b');
        gate_h.LineWidth = 3;
        hold on
    end
else
    % No Gate. Carry On.
end
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate Waypoints
for k = 1:size(obj.x,2)
    [x_arrow, y_arrow, z_arrow] = frame_builder(obj.x(:,k));
    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

    h_fr = plot3(x,y,z,'linewidth',2);

    % Set the Correct Colors
    h_fr(1).Color = [1 0 0];
    h_fr(2).Color = [0 1 0];
    h_fr(3).Color = [0 0 1];
    hold on
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set Aspect Ratio
daspect([1 1 1])

switch view_point
    case 'persp'
        view(320,20);
%              zoom(1.8)
    case 'back'
        view(-90,0);
    case 'side'
        view(0,0);
    case 'top'
        view(0,90);
    case 'nice'
        view(-60,20);
end

% Set Limits
xlim(map.x_lim);
ylim(map.y_lim);
zlim(map.z_lim);

set(gcf,'color','w');

end
