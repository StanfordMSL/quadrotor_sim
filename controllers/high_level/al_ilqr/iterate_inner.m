function al = iterate_inner(al,map,cost_param,model)

% Objective of inner loop is minimize the cost function given fixed
% lagrange multiplier or max iteration reached.

%% Unpack Some Usefull Stuff

% Constraints
con    = al.con;
lambda = al.lambda;
I_mu   = al.I_mu;

% Current Cost and previous cost.
J = cost_calc(al.x,al.u,con,lambda,I_mu,cost_param);
J_p = J;

%% Run the Inner loop
itrs = 0;
itrs_max = 30;
tol_J = 1e-2;

inner_flag  = true;       % flag true if still minimizing.
while inner_flag
%     rho = 0.01;
    rho = 0.0001;

    stab_flag = false;
    while stab_flag == false
        % Reset backward pass
        rho = 2*rho;

        % Generate candidate x and u
        x_cdd = al.x;
        u_cdd = al.u;

        % Update bp terms (l and L) and the cost predictor (del_V)
        cdd = backward_pass(x_cdd,u_cdd,al,rho,cost_param);    

        % Forward Pass Line Search
        [al_cdd,J_cdd,stab_flag] = forward_pass(al,cdd,cost_param,J_p,map,model); 
    end

    % Trajectory is stable. Solution can updated using the candidate
    % Update Traj
    al = al_cdd;
    J_p = J;
    J = J_cdd;

    % Loop Breaking Conditions
    inner_flag = inner_flag_check(J.tot,J_p.tot,tol_J,itrs,itrs_max);    

%     % Debug
%     nominal_plot(al.x,map,10,'top')
end

end