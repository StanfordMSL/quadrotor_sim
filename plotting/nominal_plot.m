function nominal_plot(x_bar,obj,step,view_point)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define plot window and clear previous stuff
figure(3)
subplot(4,4,[2:4,6:8,10:12,14:16])
cla
set(gca,'ColorOrder','factory')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Unpack some stuff
N_gt = size(obj.kf.gt,2);
map  = obj.map.act;

% Generate flight room map
if N_gt > 0
    for k = 1:N_gt
        % Gate(s) Present. Render.
        gt_dim = obj.db(obj.kf.gt(1,k)).gt_dim;
        
        q_star = quatconj(obj.kf.gt(5:8,k)');

        edges = obj.kf.gt(2:4,k)+quatrotate(q_star,gt_dim')';
            
        gate_h = plot3(edges(1,:)',edges(2,:)',edges(3,:)','b');
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
xlim(map(1,:));
ylim(map(2,:));
zlim(map(3,:));

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
