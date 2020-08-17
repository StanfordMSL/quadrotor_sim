function [traj_s, al, J_ref] = iterate_inner(traj_s,al,obj,wts,model,compute_type)

% Some preparation for loop
obj_s = obj_s_builder(traj_s.x_bar(:,1),obj);
[traj_s,al,~] = forward_pass(traj_s,obj_s,model,wts,al,'ideal');

% Initialize J_ref
J_ref.stg = 1e9;
J_ref.aug = 1e9;
J_ref.tot = J_ref.stg + J_ref.aug;

% Run inner loop
itrs = 0;
itrs_max = 10;
inner_flag = true;
while inner_flag
    [traj_s.l,traj_s.L]   = backward_pass(traj_s.x_bar,traj_s.u_bar,obj_s,model,wts,al);
    [traj_s_cand,al_cand,J_cand] = forward_pass(traj_s,obj_s,model,wts,al,'ideal');    
        
%     nominal_plot(traj_s_cand,obj,5,'persp')
%     motor_debug(traj_s_cand.u_bar,model)

    % Iteration Limit Check
    itrs = itrs + 1;
    if itrs > itrs_max
        disp(['[iterate_inner]: Hit max iteration (',num2str(itrs_max),'). Taking our best candidate']);
        inner_flag = false;
    end
    
    % Iteration Update Check
    if J_cand.tot < J_ref.tot
        traj_s = traj_s_cand;
        al = al_cand;
        J_ref = J_cand;
    else
        switch compute_type
            case 'online'
                disp(['[iterate_inner]: Leaving loop after local minima at iteration ',num2str(itrs-1)]);
                inner_flag = false;
            case 'offline'
                % carry on
        end
    end 
    
end


% % Publish some useful stuff
% J_aug = J_ref.aug;
% J_stg = J_ref.stg;
% disp(['[iterate_inner]: Aug Cost = ',num2str(J_aug), '  ||  LQR Cost = ',num2str(J_stg),'  ||  Total = ',num2str(J_ref.tot)]);

end