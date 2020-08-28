function traj = iterate_outer(traj,obj,wts_db,model)

% Objective of outer loop is to update lagrange multipliers until
% constraints are satisfied or max iteration reached.

%% Generate the Augmented Lagrangian Variables

% Initialize Variables
al = al_init(traj,obj,model);

%% Generate Cost Variables

% Generate Cost Parameters
cost_param = cost_assembly(traj,wts_db,obj);

%% Run the Outer loop
% Initialize loop parameters. 
itrs     = 0;
itrs_max = 30;
tol_con  = 1e-1;

outer_flag  = true;       % flag true if constraints are violated.
while outer_flag == true
    % Update iterate count and check for break limit
    itrs =  itrs + 1;
    
    % Update augmented lagrangian terms
    if itrs > 0
        al = al_update(al);
    end

    % Run the inner loop (minimizing L_A)
    [traj,al,J_upd] = iterate_inner(traj,al,obj,cost_param,model);

    % Loop Breaking Conditions
    outer_flag = outer_flag_check(al.con,tol_con,itrs,itrs_max);
end


disp(['[iterate_outer]: Updated Cost: ',num2str(round(J_upd.tot,5))]);

end