function traj_s = iterate_outer(traj_s,al,obj,wts,model,itr_type)

% TODO: consider other iteration methods like time (tic/toc) based. this is
% because we don't need to solve for convergence online at each iLQR
% update.
switch itr_type
    case 'fast'     % break loop on first iteration with satisfied constraints
        con_check = 1;

        while con_check > 0
            [traj_s, al, con_check,~] = iterate_inner(traj_s,al,obj,wts,model,itr_type);
        end
    case 'reps'     % break loop after # reps of satisfied constraints
        itrs = 0;
        reps = 5;
        J_ref_stg = 1e9;
        
        traj_s_cand = traj_s;
        al_cand = al;
        while itrs < reps
            [traj_s_cand, al_cand, con_check,J_cand] = iterate_inner(traj_s_cand,al_cand,obj,wts,model,itr_type);
            
            if con_check <= 0
                if J_cand.stg <= J_ref_stg
                    traj_s = traj_s_cand;
                    J_ref_stg = J_cand.stg;
                end
                itrs = itrs + 1;
            end
        end
end


end