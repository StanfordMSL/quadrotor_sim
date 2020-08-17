function traj_s = iterate_outer(traj_s,obj,wts,model,compute_type)

% Useful parameters
itrs_max  = 100;
tol_con   = 1e-2;    % addition tolerance
tol_J_aug = 1e9;

% Initialize Augmented Lagrangian
al = al_init(traj_s,22);

% Save values in the event of a failed compute
traj_s_save = traj_s;
al_save     = al;

itrs = 0;
con_flag = true;
while con_flag
    % Update iterate count and check for break limit
    itrs =  itrs + 1;
    if itrs > itrs_max
        disp(['[iterate_outer]: [ABORT] Reached max outer loop iteration count (',num2str(itrs_max),')']);
        return
    end
    
    % Update augmented lagrangian terms and iterator loop check
    if itrs > 0
        al = al_update(al);
    end
    
    switch compute_type 
        case 'offline'      % Try every means possible to get a solution.
            % Line Search the Update on Phi
            J_valid_flag = true;
            while J_valid_flag           

                [traj_s,al,J] = iterate_inner(traj_s,al,obj,wts,model,'online');
                                
                if J.aug > tol_J_aug
                    al_save.phi = al_save.phi_upd .* al_save.phi;
                    traj_s = traj_s_save;
                    al     = al_save;
                    
                    itrs   = 0;
                    
                    if any(al.phi <= 1)
                        disp('[iterate_outer]: [ABORT] Update schedule (phi) below 1.');
                        return
                    else
                        disp(['[iterate_outer]: Phi schedule too large. Reducing by factor of ',num2str(al.phi_upd)]);
                    end
                else
                    J_valid_flag = false;
                end
            end
        case 'online'       % Break loop on first iteration with satisfied constraints.
            n_con = size(al.phi,1);
            al.phi   = 1.5.*ones(n_con,1);      % keep schedule small as we assume we are already close to feasible

            [traj_s,al,J] = iterate_inner(traj_s,al,obj,wts,model,compute_type);
            
    end
    
    [con_flag,con_status] = con_check(al,tol_con);
    disp(['[iterate_outer]: Costs [aug/LQR]: ',mat2str([round(J.aug,2) round(J.stg,2)])]);
    disp(['[iterate_outer]: Unsatisfied Constraints [motor/gate/rate]: ',mat2str(con_status)]);
end

% TODO: consider other iteration methods like:
% - time (tic/toc) based
% - extra reps after convergence to produce better solutions

end