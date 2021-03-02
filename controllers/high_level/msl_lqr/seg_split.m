function traj_s = seg_split(k_now,traj)  
    traj_s.x = traj.x(:,k_now:end); 
    traj_s.u = traj.u(:,k_now:end); 
    traj_s.l = traj.l(:,k_now:end); 
    traj_s.L = traj.L(:,:,k_now:end); 
end