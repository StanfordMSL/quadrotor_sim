function traj = seg_comb(n_wp,traj,traj_s)
    fr_i = traj.kf_seg(n_wp);
    fr_f = traj.kf_seg(n_wp+1);
        
    traj.x_bar(:,fr_i:fr_f) = traj_s.x_bar; 
    traj.u_bar(:,fr_i:fr_f-1) = traj_s.u_bar;  
        
    traj.l(:,:,fr_i:fr_f-1) = traj_s.l; 
    traj.L(:,:,fr_i:fr_f-1) = traj_s.L; 
    
    traj.J(1,n_wp) = traj_s.J;
    traj.stg(1,fr_i:fr_f) = traj_s.J_stg;
    traj.aug(1,fr_i:fr_f) = traj_s.J_aug;
end