function traj = seg_comb(k_now,traj,traj_s)
    traj.x(:,k_now:end) = traj_s.x; 
    traj.u(:,k_now:end) = traj_s.u;  
        
    traj.l(:,k_now:end) = traj_s.l; 
    traj.L(:,:,k_now:end) = traj_s.L;
end