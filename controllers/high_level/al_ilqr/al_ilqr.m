function traj = al_ilqr(traj,obj,map,cost_mode,input_mode,model)

% Generate Cost Variables
cost_param = cost_assembly(traj,obj,cost_mode,input_mode,model);

% Initialize Augmented Lagrangian Variables
al = al_init(traj,obj,map,input_mode,model);

%% Run the Outer loop
% Initialize loop parameters. 
itrs     = 0;
itrs_max = 30;
tol_pos  = 1e-1;
tol_mot  = 1e-1;

outer_flag  = true;       % flag true if constraints are violated.
while outer_flag == true
    % Update iterate count and check for break limit
    itrs =  itrs + 1;
    
    % Update augmented lagrangian terms
    if itrs > 1
        al = al_update(al);
    end

    % Run the inner loop (minimizing L_A)
    [traj,al,J_upd] = iterate_inner(traj,al,obj,cost_param,model);

    % Loop Breaking Conditions
    outer_flag = outer_flag_check(al.con,tol_pos,tol_mot,itrs,itrs_max);
end