function tau_plot(x_tau,wp_tau,vel_f,n_x,n_wp,tau_dot,wp_show)

% Initialize the plotting area
figure(1)
clf
grid on
hold on
daspect([1 1 1])
view(320,20);

% Initialize some useful parameters
N = size(x_tau,3);
n = size(tau_dot,1);
legend_array = strings(n,1);

% Plot trajectories
for k  =1:N
    n_curr = n_x(k,1);
    plot3(x_tau(1,1:n_curr,k),x_tau(2,1:n_curr,k),x_tau(3,1:n_curr,k),'linewidth',1);
    
    a = num2str(tau_dot(k));
    b = num2str(vel_f(k));
    legend_array(k) = ['k=' a ' (v_f=' b ')'];
end

% Plot the waypoints
for k = 1:N
    switch wp_show
        case 'show'
            % Plot the Waypoints
            for m = 1:n_wp(k)
                [x_arrow, y_arrow, z_arrow] = frame_builder(wp_tau(:,m));
                x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
                y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
                z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

                h_wp = plot3(x,y,z,'linewidth',2);

                % Set the Correct Colors
                h_wp(1).Color = [1 0 0];
                h_wp(2).Color = [0 1 0];
                h_wp(3).Color = [0 0 1];
            end
        case 'hide'
    end
end

% Labelling
legend(legend_array,'Location','northwest');

xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');

xlim([-5 1]);
ylim([-2 2]);
zlim([ 0.0 3.0]);