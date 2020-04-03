function wp = wp_init(traj,tf,plot_show)

base_gate = [ 0.00  0.00  0.00  0.00  0.00;...
             -0.30 -0.30  0.30  0.30 -0.30;...
             -0.15  0.15  0.15 -0.15 -0.15];
         
base_pole = [ 0.00  0.00  0.00  0.00  0.00;...
             -0.03 -0.03  0.03  0.03 -0.03;...
              0.00  0.20  0.20  0.00  0.00];
map = [0 0 0]';

x = zeros(13,1);

switch traj
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'hover'
        x(:,1) = [0 ; 0 ; 1 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [0 ; 0 ; 1 ; zeros(3,1) ; 1 ; zeros(6,1)];

        t = linspace(0,tf,size(x,2));
        
        Q_key = [6 6];
        R_key = [1 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'climb'
        x(:,1) = [0 ; 0 ; 0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [0 ; 0 ; 1 ; zeros(3,1) ; 1 ; zeros(6,1)];
        
        t = linspace(0,tf,size(x,2));
        
        Q_key = [6 7];
        R_key = [1 1];             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'line'
        x(:,1) = [0 ; 0 ; 0; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [0 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,3) = [0 ; 1 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];

        t = linspace(0,tf,size(x,2));

        Q_key = [6 7;...
                 6 7];
        R_key = [1 1;...
                 1 1];                                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'half-square'
        x(:,1) = [0 ; 0 ; 0; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [0 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,3) = [0 ; 2 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,4) = [2 ; 2 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        
        t = linspace(0,tf,size(x,2));
        
        Q_key = [6 6;...
                 6 6;...
                 6 6];
        R_key = [1 1;...
                 1 1;...
                 1 1];             

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'square'        
        x(:,1) = [ 0 ; 0 ; 0; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 0 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,3) = [ 0 ; 2 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,4) = [ 2 ; 2 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,5) = [ 2 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,6) = [ 0 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,7) = [ 0 ; 0 ; 0; zeros(3,1) ; 1 ; zeros(6,1)];

        t = linspace(0,tf,size(x,2));

        Q_key = [6 6;...
                 6 6;...
                 6 6;...
                 6 6;...
                 6 6;...
                 6 6];
        R_key = [1 1;...
                 1 1;...
                 1 1;...
                 1 1;...
                 1 1;...
                 1 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'flip'        
        x(:,1) = [ 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; zeros(3,1)];
        x(:,2) = [ 0.0 ; 0.0 ; 2.0 ; 0.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0 ; 0 ; 0];
        x(:,3) = [ 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; zeros(3,1)];

        t = [0 tf/2 tf];

        Q_key = [1 3;...
                 3 7];...
        R_key = [1 1;...
                 1 1];      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'slit'        
        raw_angles = [-pi/4 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        map = bRw*base_gate + [0 0 1]';

        x(:,1) = [-1.0 ; 0.0 ; 1.0 ; zeros(3,1) ; 1.0 ; zeros(6,1)];
        x(:,2) = [ 0.0 ; 0.0 ; 1.0 ; 1.0 ; 0.0  ; 0.0 ; quat ; zeros(3,1)];    % Tilt (through the hole)
        x(:,3) = [ 1.0 ; 0.0 ; 1.0 ; zeros(3,1) ; 1.0 ; zeros(6,1)];
              
        t = linspace(0,tf,size(x,2));

        Q_key = [4 6;...
                 1 6];
        R_key = [1 1;...
                 1 1];               
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    case 'dive'        
        raw_angles = [0 -pi/4 0];
        quat = eul2quat(raw_angles)';
        map = base_pole + [0 0 0]';
        
        x(:,1) = [-2.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0  ; 0.0 ;  0.0 ; 0.0 ; zeros(3,1)];
        x(:,2) = [ 0.0 ; 0.0 ; 0.2 ; 2.0 ; 0.0 ; 0.0 ; quat ; zeros(3,1)];
        x(:,3) = [ 2.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0  ; 0.0 ;  0.0 ; 0.0 ; zeros(3,1)];
        
        t = linspace(0,tf,size(x,2));

        Q_key = [6 7;...
                 6 7];
        R_key = [1 1;...
                 1 1];       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    case 'dive vII'        
        raw_angles = [0 -pi/2 0];
        quat = eul2quat(raw_angles)';
        map = base_pole + [0 0 0]';
        
        x(:,1) = [-2.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0  ; 0.0 ;  0.0 ; 0.0 ; zeros(3,1)];
        x(:,2) = [ 0.0 ; 0.0 ; 0.2 ; 2.0 ; 0.0 ; 0.0 ; quat ; zeros(3,1)];
        x(:,3) = [ 2.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0  ; 0.0 ;  0.0 ; 0.0 ; zeros(3,1)];
        
        t = linspace(0,tf,size(x,2));

        Q_key = [1 3;...
                 3 7];
        R_key = [1 1;...
                 1 1]; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    case 'calicadabra'   
        raw_angles = [pi/8 0 0];

        left = eul2quat(-raw_angles)';
        right = eul2quat(raw_angles)';
        map = base_pole + [0 0 0]';
        
        x(:,1)   = [ 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0  ; 0.0 ;  0.0 ; 0.0 ; zeros(3,1)];
        x(:,2)   = [ 0.0 ; 0.0 ; 0.95; 0.0 ; 0.0 ; 0.0 ; left ; zeros(3,1)];
        x(:,3)   = [ 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0  ; 0.0 ;  0.0 ; 0.0 ; zeros(3,1)];
        x(:,4)   = [ 0.0 ; 0.0 ; 0.95; 0.0 ; 0.0 ; 0.0 ; right; zeros(3,1)];
        x(:,5)   = [ 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0  ; 0.0 ;  0.0 ; 0.0 ; zeros(3,1)];
        x(:,6)   = [ 0.0 ; 0.0 ; 0.95; 0.0 ; 0.0 ; 0.0 ; left ; zeros(3,1)];
        x(:,7)   = [ 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0  ; 0.0 ;  0.0 ; 0.0 ; zeros(3,1)];
        x(:,8)   = [ 0.0 ; 0.0 ; 0.95; 0.0 ; 0.0 ; 0.0 ; right; zeros(3,1)];
        x(:,9)   = [ 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0  ; 0.0 ;  0.0 ; 0.0 ; zeros(3,1)];
        
        t =  round(linspace(0,tf,9),2);

        Q_key = [6.*ones(8,1) 7.*ones(8,1)];
        R_key = ones(8,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    case 'horizon'
        map = base_pole + [0 0 0]';

        x(:,1) = [ 0.0 ; 0.0 ; 0.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; zeros(3,1)];
        x(:,2) = [ 0.0 ; 0.0 ; 0.0 ; 0.0 ; 0.0 ; 0.0 ; 1.0 ; 0.0 ; 0.0 ; 0.0 ; zeros(3,1)];
        
        t = [0 tf];

        Q_key = [1 2];
        R_key = [1 1];        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    otherwise
        disp('[wp_init]: No trajectory loaded!');
end

wp.gate = base_gate;
wp.t = t;
wp.x = x;
wp.N_wp = size(x,2);
wp.tf = tf;

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
            quat = x(7:10,k);
            bRw = quat2rotm(quat');

            pos = x(1:3,k);

            x_arrow = [pos pos+(bRw*vect_x)];
            y_arrow = [pos pos+(bRw*vect_y)];
            z_arrow = [pos pos+(bRw*vect_z)];

            pos_x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
            pos_y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
            pos_z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

            quad_h = plot3(pos_x,pos_y,pos_z,'linewidth',2);
            quad_h(1).Color = [1 0 0];
            quad_h(2).Color = [0 1 0];
            quad_h(3).Color = [0 0 1];
            
            text(pos(1),pos(2),pos(3),num2str(k));
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
