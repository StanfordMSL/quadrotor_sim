function traj = traj_init(N_traj,x_init,hover_u)    
    traj.u = hover_u.*ones(4,N_traj-1);
    traj.x = repmat(x_init,1,N_traj);
    
    traj.l = zeros(4,N_traj-1);
    traj.L = zeros(4,13,N_traj-1);
    
    traj.alpha = 1;
end