function traj = msl_lqr(traj,al,obj,wts,model)
    tic
    N_wp = size(obj.wp_arr,2)-1;

    for n_wp = 1:N_wp
        obj.wp_curr = n_wp+1;
        traj_s = seg_split(n_wp,traj);
        traj_s = iterate_outer(traj_s,al,obj,wts,model,'full');
        traj = seg_comb(n_wp,traj,traj_s);
    end
end

