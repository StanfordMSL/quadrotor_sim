function traj_s = iterate_outer(traj_s,al,obj,wts,model,bk_type)
t_max_out = 10;     % max runtime (seconds)
rep_max   = 10;     % max repetitions/loops
con_check = 1;

switch bk_type
    case 'time'
        runtime = 0;
        tic
        while runtime < t_max_out
            [traj_s, al, con_check] = iterate_inner(traj_s,al,obj,wts,model);
            
            runtime = toc;
        end
    case 'reps'
        for k = 1:rep_max
            [traj_s, al, con_check] = iterate_inner(traj_s,al,obj,wts,model);
        end
    case 'full'
        while con_check > 0
            [traj_s, al, con_check] = iterate_inner(traj_s,al,obj,wts,model);
        end
end

disp(['[iterate_outer]: Constraints Violated = ',num2str(con_check)]);

end