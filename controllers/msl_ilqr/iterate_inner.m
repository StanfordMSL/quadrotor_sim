function [traj, al, J] = iterate_inner(traj,al,obj,cost_param,model)

% Objective of inner loop is minimize the cost function given fixed
% lagrange multiplier or max iteration reached.

%% Unpack Some Usefull Stuff

% Count
N = size(traj.x,2);

del_u = zeros(4,N-1);

% Current Cost and previous cost.
J = cost_calc(traj.x,traj.u,al.con,al.lambda,al.I_mu,cost_param);
J_p = J;

%% Run the Inner loop
itrs = 0;
itrs_max = 30;
tol_J = 1e-1;

inner_flag  = true;       % flag true if still minimizing.
while inner_flag
    rho = 0.0001;
    stab_flag = false;
    while stab_flag == false
        % Reset backward pass
        del_u_c = del_u;  
        rho = 10*rho;

        % Update u_bar
        traj_c.u = traj.u + del_u_c;

        % Update bp terms (l and L) and the cost predictor (del_V)
        [traj_c.l,traj_c.L,del_V] = backward_pass(traj.x,traj.u,al,rho,cost_param,model);    

        % Ready the x for forward pass
        traj_c.x = traj.x;
        
        % Forward Pass Line Search
        [traj_c.x,del_u_c,traj_c.alpha,stab_flag] = forward_pass(traj_c,al,obj,del_V,J_p,cost_param,model,'ideal');    
    end

    % Trajectory is stable. Solution can updated using the candidate
    % Some u consistency stuff
    del_u = del_u_c;

    % Update Traj
    traj.x = traj_c.x;
    traj.u = traj_c.u;
    traj.l = traj_c.l;
    traj.L = traj_c.L;
    traj.alpha = traj_c.alpha;
 
    % debug
    nominal_plot(traj.x,obj,10,'persp')
    motor_debug(traj.u,model)

    % Update constraints struct
    [al.con,al.con_x,al.con_u] = con_compute(traj.x,traj.u ,obj,model);
    al.I_mu = con_trigger(al.con,al.lambda,al.mu);

    % Update cost and previous cost
    J_p = J;
    J   = cost_calc(traj.x,traj.u,al.con,al.lambda,al.I_mu,cost_param);

    % Loop Breaking Conditions
    inner_flag = inner_flag_check(J.tot,J_p.tot,tol_J,itrs,itrs_max);
end

% disp(['[iterate_outer]: Costs [aug/LQR]: ',mat2str([round(J_s.aug,5) round(J_s.lqr,5)])]);
% disp(['[iterate_outer]: Unsatisfied Constraints [motor/gate]: ',mat2str(con_status)]);
end