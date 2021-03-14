function traj = diff_flat_ws(obj,map,model,n_der,nom_show)

% Unpack Some Terms
fmu_dt  = model.dt_fmu;
N_wp    = size(obj.x,2);

% Trajectory time setup. We assume a constant velocity of around 2.0m/s in
% between waypoints to generate an estimated time.
t_wp = zeros(1,N_wp);
for k = 1:N_wp-1
    s_int = norm(obj.x(1:3,k+1) - obj.x(1:3,k));
    t_int = fmu_dt*round(s_int/(fmu_dt*2.0));
    
    if s_int == 0   % hover (default)
        t_wp(1,k+1) = 5;
    else
        t_wp(1,k+1) = t_wp(1,k) + t_int;
    end
end

% Convert objectives to flat outputs
[t_sigma, sigma, con_sigma] = obj2sigma(obj,map,t_wp,model,n_der);

% Solve the Piecewise QP
f_out = piecewise_QP(t_sigma,sigma,con_sigma,fmu_dt);

% Run through the model in perfect conditions to get x.
N_tr = size(f_out,3);
x_bar = zeros(13,N_tr);
x_bar(:,1) = obj.x(:,1);

u_w = zeros(4,N_tr-1);
u_m = zeros(4,N_tr-1);

for k = 1:N_tr-1
    u_w(:,k) = df_con(f_out(:,:,k),model);
    
    % Directly convert wrench to motor inputs
    T_motor = model.m2w_est_inv*u_w(:,k);
    u_m(:,k) = sqrt(T_motor./model.kw_est(1,1));
    
    FT_ext = zeros(6,1);
    x_bar(:,k+1) = quadcopter(x_bar(:,k),u_m(:,k),model,FT_ext,'fmu_ideal');
end

traj.x = x_bar;
traj.u_w = u_w;
traj.u_m = u_m;

traj.l = zeros(4,1,N_tr-1);
traj.L = zeros(4,13,N_tr-1);

traj.f_out = f_out;

% Publish some diagnostics
switch nom_show
    case 'show'
        nominal_plot(traj.x,obj,map,100,'persp');
        mthrust_debug(traj.u_m,model)
    case 'hide'
end

figure(5)
clf
plot3(squeeze(f_out(1,1,:)),squeeze(f_out(2,1,:)),squeeze(f_out(3,1,:)))
hold on
plot3(x_bar(1,:),x_bar(2,:),x_bar(3,:))
xlim([-5 5]);
ylim([-5 5]);
zlim([0 5]);
end
