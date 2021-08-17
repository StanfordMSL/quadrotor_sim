function f_out = piecewise_QP(fwp,fmu_dt)

% Tuning Parameters
mu_r = 1;
mu_psi = 1e-9;

% Unpack some stuff
t_sig = fwp.t; 
sig   = fwp.sigma;
t0 = t_sig(1);
t1 = t_sig(end);
N_fr = size(sig,3) - 2;
N_dr = size(sig,2);
N_dt = round(t_sig(end)/fmu_dt + 1);
N_H   = 4*N_dr;
N_eq = (3*5*2)+(1*3*2)+(N_fr*3);

% Initialize Output
f_out = zeros(4,5,N_dt);
f_out(:,:,1) = sig(:,1:5,1);

% Solve the QP in a Piecewise Fashion
H   = zeros(N_H,N_H);
Aeq = zeros(N_eq,N_H);
beq = zeros(N_eq,1);

k_h = 1;
for k_sig = 1:4
    idx1 = (k_sig-1)*N_dr+1;
    idx2 = k_sig*N_dr;
    
    % Load Cost Matrices and Inequalities
    if k_sig == 4       % sigma = psi   
        H(idx1:idx2,idx1:idx2) = mu_psi.*H_psi(t0,t1);   
        
        Aeq0 = [zeros(3,(k_sig-1)*N_dr) Ad02(t0) zeros(3,(4-k_sig)*N_dr)];
        Aeq1 = [zeros(3,(k_sig-1)*N_dr) Ad02(t1) zeros(3,(4-k_sig)*N_dr)];
        
        beq0 = sig(k_sig,1:3,1)';
        beq1 = sig(k_sig,1:3,end)';
        
        Aeq_fr = zeros(0,N_H);
        beq_fr = zeros(0,1);
        
    else                % sigma = r                            
        H(idx1:idx2,idx1:idx2) = mu_r.*H_r(t0,t1);
                
        Aeq0 = [zeros(5,(k_sig-1)*N_dr) Ad04(t0) zeros(5,(4-k_sig)*N_dr)];
        Aeq1 = [zeros(5,(k_sig-1)*N_dr) Ad04(t1) zeros(5,(4-k_sig)*N_dr)];
        
        beq0 = sig(k_sig,1:5,1)';
        beq1 = sig(k_sig,1:5,end)';
        
        Aeq_fr = zeros(N_fr,N_H);
        beq_fr = zeros(N_fr,1);
        for k_fr = 1:N_fr
            Aeq_fr(k_fr,:) = [zeros(1,(k_sig-1)*N_dr) Ad00(t_sig(1,k_fr+1)) zeros(1,(4-k_sig)*N_dr)];
            beq_fr(k_fr,:) = sig(k_sig,1,k_fr+1)';
        end
    end
    
    n_eq = size(Aeq0,1) + size(Aeq1,1) + size(Aeq_fr,1);

    Aeq(k_h:k_h+n_eq-1,:) = [Aeq0 ; Aeq1 ; Aeq_fr];
    beq(k_h:k_h+n_eq-1,1) = [beq0 ; beq1 ; beq_fr];
    
    k_h = k_h + n_eq + 1;
end
% Solve the QP
c = quadprog(H,[],[],[],Aeq,beq);

% Roll the solution forward in time to geneate flat outputs
for k_roll = 1:N_dt
    t_now = k_roll.*fmu_dt;
    
    A_part = Ad04(t_now);
    A_full = blkdiag(A_part,A_part,A_part,A_part);
    
    f_raw = A_full*c;
    
    for k_sig = 1:4
        idx1 = (k_sig-1)*5+1;
        idx2 = idx1+4;
        f_out(k_sig,:,k_roll) = f_raw(idx1:idx2,1)';
    end
end


end
