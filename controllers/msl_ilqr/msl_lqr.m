function traj = msl_lqr(k_now,traj,obj,wts_db,model,compute_type)
    
    % Extract trajectory segment based on current position and targer waypoint.
    switch compute_type
        case 'offline'
            N_wp = size(obj.wp_arr,2)-1;

            for n_wp = 1:N_wp
                obj.wp_curr = n_wp+1;
                fr_i = obj.kf_seg(n_wp);
                fr_f = obj.kf_seg(n_wp+1);
                
                traj_s = seg_split(fr_i,fr_f,traj);                
            end
        case 'online'
            fr_i = k_now;
            obj.wp_curr = find(obj.kf_seg > fr_i,1);
            fr_f = obj.kf_seg(obj.wp_curr);
            
            traj_s = seg_split(fr_i,fr_f,traj);            
    end
    
    % Determine current constraints
    obj_s = obj_s_builder(traj_s.x_bar(:,1),obj);

    % Iterate for segment
    traj_s = iterate_outer(traj_s,obj_s,wts_db,model);

    % Pack segment back
    traj = seg_comb(fr_i,fr_f,traj,traj_s);
end

