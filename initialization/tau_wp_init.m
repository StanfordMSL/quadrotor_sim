function wp = tau_wp_init(x0,tau_dot,dt,plot_show)

% Populate Default Gate and Pole positions to satisfy waypoint format
base_gate = [ 0.00  0.00  0.00  0.00  0.00;...
             -0.30 -0.30  0.30  0.30 -0.30;...
             -0.15  0.15  0.15 -0.15 -0.15];
         
base_pole = [ 0.00  0.00  0.00  0.00  0.00;...
             -0.03 -0.03  0.03  0.03 -0.03;...
              0.00  0.20  0.20  0.00  0.00];
map = base_pole + [0 0 0]';

% Starting/Ending Pose
pos0  = x0(1:3,1);
vel0  = x0(4:6,1);

targ_pos   = [0 ; 0 ; 0.3];
targ_angle = [0 0 0];
targ_quat  = eul2quat(targ_angle)';

% Determine Time Variables for Trajectory
tf_ideal = max(-1./(tau_dot.*vel0./pos0));

N = ceil(tf_ideal/dt)+1;
tf_actual = (N-1)*dt;
t = linspace(0,tf_actual,N);

% Determine Pos/Vel Variables for Trajectory
x = zeros(13,N);

for k = 1:N
    x_c     = pos0.*(1+tau_dot.*(vel0./pos0).*t(k)).^(1./tau_dot);
    x_dot_c = vel0.*(1+tau_dot.*(vel0./pos0).*t(k)).^((1-tau_dot)./tau_dot);
    
    for n = 1:3
        if ~isreal(x_c(n,1)) || isnan(x_c(n,1))
            x_c(n,1) = 0;
        end
        
        if ~isreal(x_dot_c(n,1))  || isnan(x_dot_c(n,1))
            x_dot_c(n,1) = 0;
        end
    end
    x(1:6,k) = [x_c+targ_pos ; x_dot_c];
end

% Determine Ang/Ang_Vel Variables for Trajectory
for k = 1:N
    if x_c == zeros(3,1)
        x(7:10,k) = targ_quat;
    else
        x(7:10,k) = [1 ; 0 ; 0 ; 0];
    end
end


% Determine Weights for Q and R
n_QR  = size(x,2) - 1;
Q_base = [6 7];
R_base = [1 1];

Q_key = repmat(Q_base,n_QR,1);
R_key = repmat(R_base,n_QR,1);    

% Pack them into wp structure
wp.gate = base_gate;
wp.t = t;
wp.x = x;
wp.N_wp = size(x,2);
wp.tf = tf_actual;

wp.x_lim = [-8.1 8.1];
wp.y_lim = [-3.2 3.2];
wp.z_lim = [0 3];

wp.Q_key = Q_key;
wp.R_key = R_key;
wp.map = map;

switch plot_show
    case 'plot'
        figure(1)
        clf
        
        gate_h = plot3(map(1,:)',map(2,:)',map(3,:)');
        gate_h.LineWidth = 3;
        hold on

        % Body Frame Axes
        vect_x = [0.2 0.0 0.0]';
        vect_y = [0.0 0.2 0.0]';
        vect_z = [0.0 0.0 0.1]';
        for k = 1:N_wp+1
            targ_quat = x(7:10,k);
            bRw = quat2rotm(targ_quat');

            pos0 = x(1:3,k);

            x_arrow = [pos0 pos0+(bRw*vect_x)];
            y_arrow = [pos0 pos0+(bRw*vect_y)];
            z_arrow = [pos0 pos0+(bRw*vect_z)];

            pos_x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
            pos_y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
            pos_z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

            quad_h = plot3(pos_x,pos_y,pos_z,'linewidth',2);
            quad_h(1).Color = [1 0 0];
            quad_h(2).Color = [0 1 0];
            quad_h(3).Color = [0 0 1];
            
            text(pos0(1),pos0(2),pos0(3),num2str(k));
        end

        xlabel('x-axis');
        ylabel('y-axis');
        zlabel('z-axis');
        grid on
        xlim([-8.1 8.1]);
        ylim([-3.2 3.2]);
        zlim([0 3]);
        daspect([1 1 1])
        view(-50,10);
    case 'no plot'
        % Do Nothing
    otherwise
        % Do Nothing
end