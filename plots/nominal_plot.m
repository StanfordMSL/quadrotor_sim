function nominal_plot(x_bar,obj,step,view_point)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define plot window and clear previous stuff
% figure(1)
% clf
figure(1)
subplot(4,4,[2:4,6:8,10:12,14:16])
cla
set(gca,'ColorOrder','factory')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate flight room map
pnts_gate_rdr = [obj.p_gc obj.p_gc(:,1)];  % render points need to terminate at start
if pnts_gate_rdr(1,1) == 999
    %carry on
else
    gate_h = plot3(pnts_gate_rdr(1,:)',pnts_gate_rdr(2,:)',pnts_gate_rdr(3,:)');
    gate_h.LineWidth = 3;
end
grid on
hold on

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

% % Plot the full trajectory
% plot3(x_bar(1,:),x_bar(2,:),x_bar(3,:),'--k','linewidth',1);

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
xlim([-3.5 3.5]);
ylim([-1.5 1.5]);
zlim([ 0.0 3.0]);

set(gcf,'color','w');

end
