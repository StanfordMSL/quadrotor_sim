function traj_s = seg_split(fr_i,fr_f,traj)  
    traj_s.x_bar = traj.x_bar(:,fr_i:fr_f); 
    traj_s.u_bar = traj.u_bar(:,fr_i:fr_f-1); 
    traj_s.l = traj.l(:,:,fr_i:fr_f-1); 
    traj_s.L = traj.L(:,:,fr_i:fr_f-1); 
end