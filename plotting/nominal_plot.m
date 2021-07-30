function nominal_plot(x_bar,gate,step,view_point)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define plot window and clear previous stuff
figure(2)
subplot(4,4,[2:4,6:8,10:12,14:16])
cla
set(gca,'ColorOrder','factory')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate flight room map
if any(ismember(fields(gate),'p_ctr'))
    N_g = size(gate.p_ctr,2);
    
    for k = 1:N_g
        % Gate(s) Present. Render.    
        p_G1 = gate.p_box(:,1,k);
        p_G2 = gate.p_box(:,2,k);
        p_G4 = gate.p_box(:,4,k);

        r_12 = p_G2 - p_G1;
        r_14 = p_G4 - p_G1;
        n_G  = cross(r_14,r_12);
        gate_dir = gate.p_box(:,1,k)+ (0.3.*n_G./norm(n_G));
        
        g_frame = [gate.p_box(:,:,k) gate.p_box(:,1,k) gate_dir];  % render points need to terminate at start

        gate_h = plot3(g_frame(1,:)',g_frame(2,:)',g_frame(3,:)','b');
        gate_h.LineWidth = 3;
        hold on
    end
else
    % No Gate. Carry On.
end
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:step:size(x_bar,2)
    [x_arrow, y_arrow, z_arrow] = frame_builder(x_bar(:,k));

%     [x_arrow, y_arrow, z_arrow] = frame_builder(x_bar(:,k));
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

% Plot the full trajectory
plot3(x_bar(1,:),x_bar(2,:),x_bar(3,:),'--k','linewidth',1);

% Set Aspect Ratio
daspect([1 1 1])

% Set Limits
xlim([-8.1 8.1]);
ylim([-3.2 3.2]);
zlim([ 0.0 3.0]);

switch view_point
    case 'persp'
        view(320,20);
%              zoom(1.8)
    case 'back'
        view(-90,0);
    case 'side'
        view(0,0);
    case 'top'
        view(-90,90);
    case 'nice'
        view(-76,7);
        campos([-5,-2,1.5]);
        camtarget([-0.02,-1,1]);
        camva(6);
end



set(gcf,'color','w');

end
