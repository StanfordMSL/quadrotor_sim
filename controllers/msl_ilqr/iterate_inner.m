function [traj_s, al, con_check] = iterate_inner(traj_s,al,obj,wts,model)
tol = 1e-2;

% Some preparation for loop
obj_s = obj_s_builder(traj_s.x_bar(:,1),obj);
[traj_s,al] = forward_pass(traj_s,obj_s,model,wts,al,'ideal');

% J_aug_prev = sum(traj_s.J_aug) + 10;
% Run inner loop
traj_s.J = traj_s.J * 999;
while 1
%     J_aug_prev = sum(traj_s.J_aug);
    [traj_s.l,traj_s.L]   = backward_pass(traj_s.x_bar,traj_s.u_bar,obj_s,model,wts,al);
    [traj_s_cand,al_cand] = forward_pass(traj_s,obj_s,model,wts,al,'ideal');    

    if traj_s_cand.J < traj_s.J
        traj_s = traj_s_cand;
        al = al_cand;
    else
        break
    end
end

% Update augmented lagrangian terms and iterator loop check
for k = 1:size(al.con,2)
    lambda_cand = al.lambda(:,k) + al.mu(:,k).*al.con(:,k);

    al.lambda(:,k) = max(zeros(22,1),lambda_cand);
    al.mu(:,k) = [al.phi_input .* al.mu(1:8,k) ;...
                   al.phi_gate .* al.mu(9:16,k)  ;...
                   al.phi_rate .* al.mu(17:22,k)];
end

con_check_input = sum(any(al.con(1:8,:) > tol));
con_check_gates = sum(any(al.con(9:16,:) > tol));
con_check_rates = sum(any(al.con(17:22,:) > tol));
con_check = con_check_input + con_check_gates + con_check_rates;


% Publish some useful stuff
% mu_debug = [al.mu(1,1) al.mu(9,1) al.mu(17,1) ]; 
% disp(['[iterate_inner]: mu (inputs gates rates)  = ',num2str(mu_debug)]);
% disp(['[iterate_inner]: Current Cost = ',num2str(traj_s.J)]);
% disp(['[iterate_inner]: Constraints Violated = ',num2str(con_check)]);

J_aug_tot = sum(traj_s.J_aug);
J_stg_tot = sum(traj_s.J_stg);
disp(['[iterate_inner]: Aug Cost = ',num2str(J_aug_tot), '  ||  LQR Cost = ',num2str(J_stg_tot),'  ||  Total = ',num2str(traj_s.J)]);
disp(['[iterate_inner]: Input || Gate || Rate Violations = ',num2str(con_check_input),'  ',num2str(con_check_gates),'   ',num2str(con_check_rates)]);
nominal_plot(traj_s,obj,50,'persp');

disp('====================================================================');

end