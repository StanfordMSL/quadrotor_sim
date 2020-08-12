function [traj_s, al, con_check, J_ref] = iterate_inner(traj_s,al,obj,wts,model,itr_type)
tol = 1e-2;

% Some preparation for loop
obj_s = obj_s_builder(traj_s.x_bar(:,1),obj);
[traj_s,al,J_ref] = forward_pass(traj_s,obj_s,model,wts,al,'ideal');

% Run inner loop
itrs_max = 10;
for itrs = 1:itrs_max
    [traj_s.l,traj_s.L]   = backward_pass(traj_s.x_bar,traj_s.u_bar,obj_s,model,wts,al);
    [traj_s_cand,al_cand,J_cand] = forward_pass(traj_s,obj_s,model,wts,al,'ideal');    

    if J_cand.tot < J_ref.tot
        traj_s = traj_s_cand;
        al = al_cand;
        
        J_ref = J_cand;
    else
        switch itr_type
            case 'fast'
                break
            case 'reps'
                % carry on
        end
    end
end
if itrs == itrs_max
    disp(['[iterate_inner]: Hit max iteration (',num2str(itrs_max),'). Taking our best candidate']);
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
J_aug = J_ref.aug;
J_stg = J_ref.stg;
disp(['[iterate_inner]: Aug Cost = ',num2str(J_aug), '  ||  LQR Cost = ',num2str(J_stg),'  ||  Total = ',num2str(J_ref.tot)]);
disp(['[iterate_inner]: Input / Gate / Rate Violations = ',num2str(con_check_input),' / ',num2str(con_check_gates),' / ',num2str(con_check_rates)]);

disp('===========================================================================================');

end