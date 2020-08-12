function traj = seg_comb(fr_i,fr_f,traj,traj_s)
    traj.x_bar(:,fr_i:fr_f) = traj_s.x_bar; 
    traj.u_bar(:,fr_i:fr_f-1) = traj_s.u_bar;  
        
    traj.l(:,:,fr_i:fr_f-1) = traj_s.l; 
    traj.L(:,:,fr_i:fr_f-1) = traj_s.L;     
end