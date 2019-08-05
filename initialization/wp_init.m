function t_kf = wp_init(traj,tf,plot_show)

switch traj
    case 'hover'
        t = [0 tf];
        
        x = zeros(12,2);
        x(:,1) = [0 ; 0 ; 1 ; zeros(9,1)];
        x(:,2) = [0 ; 0 ; 1 ; zeros(9,1)];
        
        N_wp = 2;
    case 'hi-res hover'
        N_wp = tf/0.5 + 1;
        t = linspace(0,tf,N_wp);
        
        x = zeros(12,N_wp);
        x(3,:) = ones(1,N_wp);        
    case 'climb'
        t = [0 tf];
        
        x = zeros(12,2);
        x(:,2) = [0 ; 0 ; 1 ; zeros(9,1)];
        
        N_wp = 2;

    case 'line'
        t = [0 0.5*tf tf];
        
        x = zeros(12,3);
        x(:,2) = [0 ; 0 ; 1; zeros(9,1)];
        x(:,3) = [1 ; 0 ; 1; zeros(9,1)];
        
        N_wp = 3;
    case 'square'
        t = [0 0.1*tf 0.3*tf 0.5*tf 0.7*tf 0.9*tf tf]';
        
        x = zeros(12,7);
        x(:,2) = [0 ; 0 ; 1; zeros(9,1)];
        x(:,3) = [0 ; 2 ; 1; zeros(9,1)];
        x(:,4) = [2 ; 2 ; 1; zeros(9,1)];
        x(:,5) = [2 ; 0 ; 1; zeros(9,1)];
        x(:,6) = [0 ; 0 ; 1; zeros(9,1)];
        x(:,7) = [0 ; 0 ; 1; zeros(9,1)];
        
        N_wp = 7;
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
    case 'slit'

    otherwise
        % Do Nothing
end

t_kf.t = t;
t_kf.x = x;
t_kf.N_wp = N_wp;

switch plot_show
    case 'plot'
        figure(1)
        clf
        % Plot Craft Angles
        vect_x = [0.3 0.0 0.0]';
        vect_y = [0.0 0.3 0.0]';
        vect_z = [0.0 0.0 0.3]';

        % Construct Rotation Matrices
        bRw = quat2rotm(x(7:10,1)');
        pos = x(1:3,1);

        x_arrow = [pos pos+(bRw*vect_x)];
        y_arrow = [pos pos+(bRw*vect_y)];
        z_arrow = [pos pos+(bRw*vect_z)];

        x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
        y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
        z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

        %     subplot(2,2,[1 2]);
        h = plot3(x,y,z,'linewidth',2);
        h(1).Color = [1 0 0];
        h(2).Color = [0 1 0];
        h(3).Color = [0 0 1];
        hold on

        snaps = 100;
        snap_steps = step_total/snaps;
        for k = 1:snaps
            index = round(k*snap_steps);

            q0 = sqrt(1-x(7:9,index)'*x(7:9,index));
            quat = [q0 ; x(7:9,index)];
            bRw = quat2rotm(quat);

            pos = x(1:3,index);

            x_arrow = [pos pos+(bRw*vect_x)];
            y_arrow = [pos pos+(bRw*vect_y)];
            z_arrow = [pos pos+(bRw*vect_z)];

            x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
            y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
            z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

            h = plot3(x,y,z,'linewidth',2);
            h(1).Color = [1 0 0];
            h(2).Color = [0 1 0];
            h(3).Color = [0 0 1];
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
