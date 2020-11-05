function traj = diff_flat_ws(traj,obj,model,nom_show)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Number of Derivative Orders
n_der = 10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Unpack Some Terms
fmu_dt  = model.dt_fmu;
N_tr    = size(traj.x,2);
N_wp    = size(obj.wp_arr,2);

% Convert objectives to flat outputs
[kf_sig, sigma] = obj2sigma(obj,n_der,N_wp,N_tr);
f_out = traj_planner(kf_sig,sigma,fmu_dt,N_tr,n_der);

% Run through the model in perfect conditions to get x.
x_bar = zeros(13,N_tr);
x_bar(:,1) = obj.wp_arr(:,1);
u_bar = zeros(4,N_tr-1);

for k = 1:N_tr-1
    u_curr = df_con(f_out(:,:,k),model);
    u_bar(:,k) = u_curr;
    
    FT_ext = zeros(6,1);
    x_bar(:,k+1) = quadcopter(x_bar(:,k),u_bar(:,k),model,FT_ext,'fmu_ideal');
end


traj.x = x_bar;
traj.u = u_bar;

traj.l = zeros(4,1,N_tr-1);
traj.L = zeros(4,13,N_tr-1);

% Publish some diagnostics
switch nom_show
    case 'show'
        nominal_plot(traj.x,obj,5,'persp');
        mthrust_debug(traj.u,model)
    case 'hide'
end

end
