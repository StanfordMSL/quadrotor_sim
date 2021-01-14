function traj = msl_lqr(k_now,traj,obj,wts_db,model)
    % Extract trajectory segment based on current position.
    traj_s = seg_split(k_now,traj);     
    
    % Iterate for segment
    traj_s = iterate_outer(traj_s,obj,wts_db,model);

    % Pack segment back
    traj = seg_comb(k_now,traj,traj_s);
end

