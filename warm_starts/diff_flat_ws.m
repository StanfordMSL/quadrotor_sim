function traj = diff_flat_ws(obj,map,model,n_der,nom_show)

% Unpack Some Terms
fmu_dt  = model.clock.dt_fmu;
N_wp    = size(obj.x,2);

% Trajectory time setup. We assume a constant velocity of around 0.5m/s in
% between waypoints to generate an estimated time.
vel = 1.0;

t_wp = zeros(1,N_wp);
for k = 1:N_wp-1
    s_int = norm(obj.x(1:3,k+1) - obj.x(1:3,k));
    t_int = fmu_dt*round(s_int/(fmu_dt*vel));
    
    if s_int == 0   % hover (default)
        t_wp(1,k+1) = 5;
    else
        t_wp(1,k+1) = t_wp(1,k) + t_int;
    end
end

% Convert objectives to flat outputs
[t_sigma, sigma, con_sigma] = obj2sigma(obj,t_wp,n_der);

% Solve the Piecewise QP
f_out = piecewise_QP(t_sigma,sigma,con_sigma,fmu_dt);

% Run through the model in perfect conditions to get x.
N_tr = size(f_out,3);
x_bar = zeros(13,N_tr);
x_bar(:,1) = obj.x(:,1);

u_wr = zeros(4,N_tr-1);

for k = 1:N_tr-1
    u_wr(:,k) = df_con(f_out(:,:,k),model.est);
    
    % Directly convert wrench to motor inputs
    T_motor = model.est.w2m*u_wr(:,k);
    u_mt = sqrt(T_motor./model.est.kw(1,1));
    
    FT_ext = zeros(6,1);
    wt = zeros(13,1);
    x_bar(:,k+1) = quadcopter_est(x_bar(:,k),u_mt,FT_ext,wt);
end

traj.x = x_bar;
traj.u = u_wr;

traj.t_fmu = t_sigma;
traj.f_out = f_out;

% Publish some diagnostics
switch nom_show
    case 'show'
        nominal_plot(traj.x,map,100,'persp');
    case 'hide'
end

end
