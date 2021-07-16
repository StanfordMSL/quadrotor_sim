function f_traj = piecewise_QP(fwp,fmu_dt)

% Unpack some stuff
t_sig = fwp.t; 
sig   = fwp.sigma;
t0 = t_sig(1);
t1 = t_sig(end);
N_dt = round(t_sig(end)/fmu_dt + 1);

% Initialize Output
f_traj = zeros(4,5,N_dt);
f_traj(:,:,1) = sig(:,1:5,1);

% Solve the QP in a Piecewise Fashion
for k_sig = 1:4
    % Load cost matrix
    if ( k_sig >= 1 ) && ( k_sig < 4 )     % sigma = r
        H = QP_H_r(t0,t1);
    else                                   % sigma = psi
        H = QP_H_psi(t0,t1);
    end

    % Generate Constraints
    t_pw = [t0 t1];

    [Aeq, beq] = QP_con_builder(t_pw,sigma_pw,con_pw);

    % Solve the QP
    c = quadprog(H,[],[],[],Aeq,beq);

    % Roll the solution forward in time to geneate flat outputs
    for t_now = t0:fmu_dt:t1  
        k_now = round((t_now/fmu_dt) + 1);

        f_out_raw = QP_A_basis(t_now-t0)*c;
        f_traj(k_sig,:,k_now) = f_out_raw(1:5)';
    end
end

end
