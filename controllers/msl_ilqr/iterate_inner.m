function [traj, al, J] = iterate_inner(traj,al,obj,cost_param,model)

% Objective of inner loop is minimize the cost function given fixed
% lagrange multiplier or max iteration reached.

%% Unpack Some Usefull Stuff

% Count
N = size(traj.x,2);

% Update variables
del_u_L = zeros(4,N-1);
u_upd = traj.u; 
x_upd = traj.x;

% Current Cost and previous cost.
J = cost_calc(traj.x,traj.u,al.con,al.lambda,al.I_mu,cost_param);
J_p = J;

%% Run the Inner loop
itrs = 0;
itrs_max = 30;
tol_J = 1e-2;

inner_flag  = true;       % flag true if still minimizing.
while inner_flag
    rho = 0.01;
    stab_flag = false;
    while stab_flag == false
        % Reset backward pass
        rho = 2*rho;

        % Update x_bar and u_bar
        traj_c.u = u_upd;
        traj_c.x = x_upd;

        % Update bp terms (l and L) and the cost predictor (del_V)
        [traj_c.l,traj_c.L,del_V] = backward_pass(traj_c.x,traj_c.u,al,rho,cost_param,model);    

        % Forward Pass Line Search
        [x_temp,del_u_l,del_u_L,stab_flag] = forward_pass(traj_c,al,obj,del_V,J_p,cost_param,model,'ideal'); 
    end

    % Trajectory is stable. Solution can updated using the candidate

    % Update Traj
    traj.x = traj_c.x;
    traj.u = traj_c.u + del_u_l;
    traj.l = traj_c.l;
    traj.L = traj_c.L;
 
    % Update constraints struct
    x_upd = x_temp;
    u_upd = traj.u + del_u_L;
    [al.con,al.con_x,al.con_u] = con_compute(x_upd,u_upd,obj,model);
    al.I_mu = con_trigger(al.con,al.lambda,al.mu);

    % Update cost and previous cost
    J_p = J;
    J   = cost_calc(x_upd,u_upd,al.con,al.lambda,al.I_mu,cost_param);

    % debug
    nominal_plot(x_upd,obj,5,'nice')
    mthrust_debug(u_upd,model)
%     con_check_gates = sum(any(al.con(7:22,:) > 1e-1))
  
    % Loop Breaking Conditions
    inner_flag = inner_flag_check(J.tot,J_p.tot,tol_J,itrs,itrs_max);
    
end

end