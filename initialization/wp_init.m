function wp = wp_init(traj,t_now,tf,plot_show)

base_gate = [ 0.00  0.00  0.00  0.00  0.00;...
             -0.30 -0.30  0.30  0.30 -0.30;...
             -0.15  0.15  0.15 -0.15 -0.15];
map = [0 0 0]';

switch traj
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'hover'
        N_wp = 2;      
        t = [0 tf];
        
        x = zeros(12,2);
        x(:,1) = [0 ; 0 ; 1 ; zeros(9,1)];
        x(:,2) = [0 ; 0 ; 1 ; zeros(9,1)];
        
        labels = {' 1',' 2'};
        
        Q_key = [1 4]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'hi-res hover'
        N_wp = tf/0.5 + 1;
        t = linspace(0,tf,N_wp);
        
        x = zeros(12,N_wp);
        x(3,:) = ones(1,N_wp);  
        
        labels = cell(N_wp,1);        
        for k = 1:N_wp
            labels{k} = [' ',num2str(k)];
        end
        
        Q_key = zeros(N_wp,1);
        Q_key(1,1) = 1;
        for k = 2:N_wp
            Q_key(k,1) = 4;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'climb'
        N_wp = 2;
        t = [0 tf];
        
        x = zeros(12,2);
        x(:,2) = [0 ; 0 ; 1 ; zeros(9,1)];
        
        labels = {' 1',' 2'};
        
        Q_key = [1 4]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'line'
        N_wp = 3;
        t = [0 0.5*tf tf];
        
        x = zeros(12,3);
        x(:,2) = [0 ; 0 ; 1; zeros(9,1)];
        x(:,3) = [0 ; 1 ; 1; zeros(9,1)];

        labels = {' 1',' 2',' 3'};
        
        Q_key = [1 4 4]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'half-square'
        N_wp = 4;
        t = [0 (1/3)*tf (2/3)*tf tf];
        
        x = zeros(12,4);
        x(:,2) = [0 ; 0 ; 1; zeros(9,1)];
        x(:,3) = [0 ; 1 ; 1; zeros(9,1)];
        x(:,4) = [0 ; 0 ; 1; zeros(9,1)];

        labels = {' 1',' 2',' 3',' 4'};
        
        Q_key = [1 4 4 4]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'square'
        N_wp = 7;
        t = [0 0.1*tf 0.3*tf 0.5*tf 0.7*tf 0.9*tf tf]';
        
        x = zeros(12,7);
        x(:,2) = [ 0 ; 0 ; 1; zeros(9,1)];
        x(:,3) = [ 0 ; 2 ; 1; zeros(9,1)];
        x(:,4) = [ 2 ; 2 ; 1; zeros(9,1)];
        x(:,5) = [ 2 ; 0 ; 1; zeros(9,1)];
        x(:,6) = [ 0 ; 0 ; 1; zeros(9,1)];
        x(:,7) = [ 0 ; 0 ; 0; zeros(9,1)];
        
        labels = {' 1',' 2',' 3',' 4',' 5','   ,6','   ,7'};
        
        Q_key = [1 4 4 4 4 4 4]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'hi-res square'
        N_wp = tf/0.5 + 1;
        t = linspace(0,tf,N_wp);
        
        kfr = 6;
        x = zeros(12,N_wp);

        for k = 1:N_wp
            if     (k >= round(0*N_wp/kfr)) && (k < round(1*N_wp/kfr))
                x(:,k) = [0 ; 0 ; 0; zeros(9,1)];
            elseif (k >= round(1*N_wp/kfr)) && (k < round(2*N_wp/kfr))
                x(:,k) = [0 ; 0 ; 1; zeros(9,1)];
            elseif (k >= round(2*N_wp/kfr)) && (k < round(3*N_wp/kfr))
                x(:,k) = [0 ; 2 ; 1; zeros(9,1)];
            elseif (k >= round(3*N_wp/kfr)) && (k < round(4*N_wp/kfr))
                x(:,k) = [2 ; 2 ; 1; zeros(9,1)];
            elseif (k >= round(4*N_wp/kfr)) && (k < round(5*N_wp/kfr))
                x(:,k) = [2 ; 0 ; 1; zeros(9,1)];
            elseif (k >= round(5*N_wp/kfr)) && (k <= round(6*N_wp/kfr))
                x(:,k) = [0 ; 0 ; 1; zeros(9,1)];
            end
        end

        labels = cell(N_wp,1);        
        for k = 1:N_wp
            labels{k} = [' ',num2str(k)];
        end
        
        Q_key = zeros(N_wp,1);
        Q_key(1,1) = 1;
        for k = 2:N_wp
            Q_key(k,1) = 4;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    case 'slit'
        N_wp = 7;
        t = [0 0.2*tf 0.5*tf-0.2 0.5*tf 0.5*tf+0.2 0.8*tf tf];
        
        x = zeros(12,N_wp);
        x(:,1) = [-2.0 ; 0.0 ; 0.0 ; zeros(9,1)];
        x(:,2) = [-2.0 ; 0.0 ; 1.0 ; zeros(9,1)];
        x(:,3) = [ 0.0 ; 0.0 ; 0.8 ; 1.0 ; 0.0 ; 0.0 ; 0.3 ; 0.0 ; 0.0 ; zeros(3,1)];
        x(:,4) = [ 0.0 ; 0.0 ; 1.0 ; 1.0 ; 0.0 ; 0.0 ; 0.3 ; 0.0 ; 0.0 ; zeros(3,1)];
        x(:,5) = [ 0.0 ; 0.0 ; 1.2 ; 1.0 ; 0.0 ; 0.0 ; 0.3 ; 0.0 ; 0.0 ; zeros(3,1)];
        x(:,6) = [ 2.0 ; 0.0 ; 1.0 ; zeros(9,1)];
        x(:,7) = [ 2.0 ; 0.0 ; 0.0 ; zeros(9,1)];
                
        labels = {' 1',' 2',' 3',' 4',' 5'};

        Q_key = [1 2 2 3 4 3 2 2]';
        
        q0 = sqrt(1-x(7:9,3)'*x(7:9,3));
        quat = [q0 ; x(7:9,3)];
        bRw = quat2rotm(quat');

        map = bRw*base_gate + [0 0 1]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    otherwise
        disp('[wp_init]: No trajectory loaded!');
end

wp.gate = base_gate;
wp.t = t_now + t;
wp.x = x;
wp.N_wp = N_wp;
wp.tf = tf;

wp.x_lim = [-8.1 8.1];
wp.y_lim = [-3.2 3.2];
wp.z_lim = [0 3];

wp.labels = labels;
wp.Q_key = Q_key;
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
        for k = 1:N_wp
            q0 = sqrt(1-x(7:9,k)'*x(7:9,k));
            quat = [q0 ; x(7:9,k)];
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
            
            text(pos(1),pos(2),pos(3),labels{k});
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
