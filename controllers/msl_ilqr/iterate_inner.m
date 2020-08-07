function [traj_s, al, con_check] = iterate_inner(traj_s,al,obj,wts,model)

% Some preparation for loop
obj_s = obj_s_builder(traj_s.x_bar(:,1),obj);
[traj_s,~] = forward_pass(traj_s,obj_s,model,wts,al,'ideal');
itr = 0;
itr_max = 10;
tol = 1e-2;

% Run inner loop
while itr < itr_max
    [traj_s.l,traj_s.L] = backward_pass(traj_s.x_bar,traj_s.u_bar,obj_s,model,wts,al);
    [traj_s,con_struct] = forward_pass(traj_s,obj_s,model,wts,al,'ideal');
    
    con_check = sum(any(con_struct.con > tol));
    if con_check == 0
        break
    else
        itr = itr + 1;
    end
end

% Update augmented lagrangian terms and iterator loop check
for k = 1:size(con_struct.con,2)
    lambda_cand = al.lambda(:,k) + al.mu(:,k).*con_struct.con(:,k);

    al.lambda(:,k) = max(zeros(16,1),lambda_cand);
    al.mu(:,k) = al.phi.*al.mu(:,k);
end

% Publish con_check

disp(['[iterate_inner]: Current Cost = ',num2str(traj_s.J)]);
disp(['[iterate_inner]: Constraints Violated = ',num2str(con_check)]);

nominal_plot(traj_s,obj,50,'persp');

end