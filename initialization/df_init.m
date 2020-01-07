function nom = df_init(wp,model,angle_axis)
tic
n_p = 15;
sigma = wp2sigma(wp,angle_axis,n_p);
[t_out, f_out] = traj_planner(sigma,model.con_hz,n_p);
count = model.con_hz*wp.tf+1;

x_bar = zeros(13,count);
x_bar(:,1) = wp.x(:,1);

u_bar = zeros(4,count);
u_bar(:,1) = model.hover_wrench;

for k = 1:count-1
    u_curr = df_con(f_out(:,:,k),model,angle_axis);
    u_bar(:,k) = u_curr;
    curr_m_cmd = wrench2m_controller(u_curr,model);
    
    FT_ext = zeros(6,1);
    x_bar(:,k+1) = quadcopter(x_bar(:,k),curr_m_cmd,model,FT_ext,'fc');
end

nom.t_bar = t_out;
nom.x_bar = x_bar;
nom.u_bar = u_bar;
nom.f_out = f_out;

N =  wp.tf*model.con_hz+1;
nom.total = N;
nom.alpha = 1;
nom.l = zeros(4,1,N-1);
nom.L = zeros(4,13,N-1);

disp(['[df_init]: Diff. Flat Compute Completed in: ',num2str(toc),' seconds.']);
