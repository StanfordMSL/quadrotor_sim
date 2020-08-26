function traj_s = iterate_outer(traj_s,obj_s,wts_db,model)

% Objective of outer loop is to update lagrange multipliers until
% constraints are satisfied.

% Initialize Augmented Lagrangian
al_s = al_init(traj_s,22,obj_s.pnts_gate,model);

% Generate cost parameters
cost_param = cost_assembly(wts_db,'initial',traj_s,obj_s,model);

% Initialize loop parameters
itrs_max  = 20;
tol_con   = 0;    % addition tolerance
itrs      = 0;
con_flag  = true;

% Run the actual loop
while con_flag
    % Update iterate count and check for break limit
    itrs =  itrs + 1;
    if itrs > itrs_max
        disp(['[iterate_outer]: [ABORT] Reached max outer loop iteration count (',num2str(itrs_max),')']);
        return
    end
    
    % Update augmented lagrangian terms and iterator loop check
    if itrs > 0
        al_s = al_update(al_s);
    end

    % Run the inner loop (minimizing L_A)
    [traj_s,al_s,J_s] = iterate_inner(traj_s,al_s,obj_s,cost_param,model);

    [con_flag,con_status] = con_check(al_s,tol_con);
    disp(['[iterate_outer]: Costs [aug/LQR]: ',mat2str([round(J_s.aug,5) round(J_s.lqr,5)])]);
    disp(['[iterate_outer]: Unsatisfied Constraints [motor/gate/rate]: ',mat2str(con_status)]);
end

end