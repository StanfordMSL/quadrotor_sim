function traj_s = seg_split(n_wp,traj)
    fr_i = traj.kf_seg(n_wp);
    fr_f = traj.kf_seg(n_wp+1);
    
    traj_s.x_bar = traj.x_bar(:,fr_i:fr_f); 
    traj_s.u_bar = traj.u_bar(:,fr_i:fr_f-1); 
    traj_s.l = traj.l(:,:,fr_i:fr_f-1); 
    traj_s.L = traj.L(:,:,fr_i:fr_f-1); 
        
    traj_s.J = traj.J(1,n_wp);
    traj_s.J_stg = traj.J_stg(1,fr_i:fr_f);
    traj_s.J_aug = traj.J_aug(1,fr_i:fr_f);
end