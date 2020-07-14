function nom = df_init(wp,model,angle_axis,nom_show)
tic

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Number of Derivative Orders
n_p = 15;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unpack Some Terms
hz = model.hz_ctl;
N = round(hz*wp.tf)+1;

% Convert waypoints to flat outputs
wp.sigma = pose2sigma(wp,angle_axis,n_p);
[t_out, f_out] = traj_planner(wp,N,hz,n_p);

% Run through the model in perfect conditions to get x.
x_bar = zeros(13,N);
x_bar(:,1) = wp.x(:,1);
u_bar = zeros(4,N-1);

% tempororary hack
model_temp  = model_init('v1.0.0','high-speed');       % Initialize quadcopter

for k = 1:N-1
    u_curr = df_con(f_out(:,:,k),model,angle_axis);
    u_bar(:,k) = u_curr;
    curr_m_cmd = wrench2m_controller(u_curr,model);
    
    FT_ext = zeros(6,1);
    
    x_bar(:,k+1) = quadcopter(x_bar(:,k),curr_m_cmd,model_temp,FT_ext,'ctl');
end

% Generate the number of ctl frames between each waypoint.
kf = zeros(1,size(wp.t,2));
for k = 1:size(wp.t,2)
    kf(1,k) = (wp.t(k)*model.hz_ctl) + 1;
end

% Initialize cost_curr_array
cc_arr_size = model.hz_lqr * t_out(end);
cost_curr = zeros(cc_arr_size,1);

% Save the stuff into nom.
nom.t_bar = t_out;
nom.x_bar = x_bar;
nom.u_bar = u_bar;
nom.f_out = f_out;

nom.total = N;
nom.l = zeros(4,1,N-1);
nom.L = zeros(4,13,N-1);

nom.kf = kf;
nom.wp_curr = 1;

nom.cost_curr = cost_curr;

% Publish some diagnostics
switch nom_show
    case 'show'
        nominal_plot(wp,nom,'persp',10);
    case 'hide'
end
disp('[df_init]: Note, orientation data is lost in the wp2sigma step');
disp(['[df_init]: Trajectory has ',num2str(wp.N_wp),' waypoints over ',num2str(wp.tf),' seconds']);
disp(['[df_init]: Diff. Flat Trajectory Computed in: ',num2str(toc),' seconds.']);
