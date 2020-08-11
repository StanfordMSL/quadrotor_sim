function traj = msl_lqr(traj,obj,wts,model)
    tic
    N_wp = size(obj.wp_arr,2)-1;

    for n_wp = 1:N_wp
        % Load Trajectory Segment
        obj.wp_curr = n_wp+1;
        traj_s = seg_split(n_wp,traj);

        % Initialize Augmented Lagrangian
        al = al_init(traj_s,16);

        % Iterate for segment
        traj_s = iterate_outer(traj_s,al,obj,wts,model,'full');
        
        % Pack segment back
        traj = seg_comb(n_wp,traj,traj_s);
    end
end

