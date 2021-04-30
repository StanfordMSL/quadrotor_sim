function f_out = piecewise_QP(t_sigma,sigma,con_sigma,fmu_dt)

% Unpack some stuff
n_wp  = size(sigma,3);

% Container variables
N_dt = round(t_sigma(end)/fmu_dt + 1);
f_out = zeros(4,5,N_dt);

f_out(:,:,1) = sigma(:,1:5,1);
sigma_start = sigma(:,:,1);
sigma_prev = sigma(:,:,1);
for k_wp = 1:n_wp-1
    % Time Parameters
    t0 = t_sigma(1,k_wp);
    t1 = t_sigma(1,k_wp+1);
    
    t_pw_0 = 0;
    t_pw_1 = t1-t0;
    
    % Solve the QP in a Piecewise Fashion
    for k_sig = 1:4
        % Load cost matrix
        if ( k_sig >= 1 ) && ( k_sig < 4 )     % sigma = r
            H = QP_H_r(t_pw_0,t_pw_1);
        else                                   % sigma = psi
            H = QP_H_psi(t_pw_0,t_pw_1);
        end
        
        % Generate Constraints
        t_pw = [t_pw_0 t_pw_1];
        sigma_pw = [sigma_start(k_sig,:)' squeeze(sigma(k_sig,:,k_wp+1))'];   
        con_pw = [ones(5,1) squeeze(con_sigma(k_sig,:,k_wp+1))'];
        [Aeq, beq] = QP_con_builder(t_pw,sigma_pw,con_pw);
        
        % Solve the QP
        c = quadprog(H,[],[],[],Aeq,beq);
        
        % Roll the solution forward in time to geneate flat outputs
        for t_now = t0:fmu_dt:t1  
            k_now = round((t_now/fmu_dt) + 1);
            
            f_out_raw = QP_A_basis(t_now-t0)*c;
            f_out(k_sig,:,k_now) = f_out_raw(1:5)';
        end
        sigma_prev(k_sig,:) = f_out_raw';
    end
    sigma_start = sigma_prev;
end

end
