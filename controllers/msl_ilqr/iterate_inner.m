function [traj_s, al_s, J_s] = iterate_inner(traj_s,al_s,obj_s,cost_param,model)

% Tolerances
tol_alpha = 1e-6;

% Assign current segment trajectory to candidate
traj_c = traj_s;
al_c   = al_s;

% Compute the segment cost, which is also the initial 'previous' cost
J_s = cost_calc(traj_s.x_bar,traj_s.u_bar,al_s.con,al_s.lambda,al_s.I_mu,cost_param);
J_p = J_s;

% Run inner loop
itrs = 0;
itrs_max = 20;
inner_flag = true;
rho = 0.01;
while inner_flag
    % Update bp terms (l and L) and the cost predictor (del_V)
    [traj_c.l,traj_c.L,del_V] = backward_pass(traj_c.x_bar,traj_c.u_bar,al_s,rho,cost_param,model);    
    
    % Forward Pass Line Search
    x_save = traj_c.x_bar;
    u_save = traj_c.u_bar;
    alpha  = 1;
    while true
        % Update fp terms (x_bar and u_bar)
        [traj_c.x_bar,traj_c.u_bar] = forward_pass(traj_c,alpha,model,'ideal');    
        
%         % Plot for debug
%         nominal_plot(traj_c.x_bar,obj_s,5,'persp')
%         motor_debug(traj_c.u_bar,model)
        
        % Update constraints and their partials
        [al_c.con,al_c.con_x,al_c.con_u] = con_compute(traj_c.x_bar,traj_c.u_bar,obj_s.pnts_gate,model);
        
        % Update the trigger matrices
        al_c.I_mu = con_trigger(al_c.con,al_c.lambda,al_c.mu);
        
        % Update the the cost function and its components
        J_c  = cost_calc(traj_c.x_bar,traj_c.u_bar,al_c.con,al_c.lambda,al_c.I_mu,cost_param);

        % Regularization check
        if ~isnan(J_c.tot)
            % Not exploding. Carry On.
        else
            % Cost exploding. Resetting and resorting to a more naive gradient
            traj_c = traj_s;
            rho = 10*rho;
%             disp('[iterate_inner]: Cost explosion. Increasing rho to force it back.');
            break
        end
        
        % Expected Improvement check
        z = (J_c.tot-J_p.tot)/(alpha*sum(del_V(1,:)) + (alpha^2)*sum(del_V(2,:)));        
        if (z > 1e-4) && (z < 3)
            % Within bounds. Carry On. Update previous cost.
            J_p = J_c;
            break
        else
            % Out of bounds. Reset
            traj_c.x_bar = x_save;
            traj_c.u_bar = u_save;
            alpha = 0.5*alpha;
            
            if alpha < tol_alpha
%                 disp(['[iterate_inner]: Exceed alpha tolerance (',num2str(tol_alpha),').']);
                break;
            end
            
%             disp(['[iterate_inner]: Out of bounds with ratio (',num2str(z),').']);
        end
    end

    % Iteration Update Check
    if J_c.tot < J_s.tot
        traj_s = traj_c;
        al_s   = al_c;
        J_s    = J_c;        
        
        % Plot for debug
        nominal_plot(traj_s.x_bar,obj_s,5,'persp')
        motor_debug(traj_s.u_bar,model)
    end
    
    % Alpha Convergence Check
    if alpha < tol_alpha
%         disp('[iterate_inner]: Converged to a pure feedback law.');
        inner_flag = false;
    end
    
    % Iteration Limit Check
    itrs = itrs + 1;
    if itrs > itrs_max
%         disp(['[iterate_inner]: Hit max iteration (',num2str(itrs_max),'). Taking our best candidate']);
        inner_flag = false;
    end
end

end