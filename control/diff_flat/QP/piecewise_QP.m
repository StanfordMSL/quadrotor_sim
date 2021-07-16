function f_out = piecewise_QP(fwp,fmu_dt)

% Unpack some stuff
t_sig = fwp.t; 
sig   = fwp.sigma;
t0 = t_sig(1);
t1 = t_sig(end);
N_fr = size(sig,3) - 2;
N_dr = size(sig,2);
N_dt = round(t_sig(end)/fmu_dt + 1);

% Initialize Output
f_out = zeros(4,5,N_dt);
f_out(:,:,1) = sig(:,1:5,1);

% Solve the QP in a Piecewise Fashion
for k_sig = 1:4  
    % Load Cost Matrices and Inequalities
    if k_sig == 4       % sigma = psi   
   
        H = H_psi(t0,t1);   
        
        Aeq0 = Ad02(t0);
        Aeq1 = Ad02(t1);
        
        beq0 = sig(k_sig,1:3,1)';
        beq1 = sig(k_sig,1:3,end)';
        
        Aeq_fr = zeros(0,N_dr);
        beq_fr = zeros(0,N_dr);
    else                % sigma = r                            
        H = H_r(t0,t1);
                
        Aeq0 = Ad04(t0);
        Aeq1 = Ad04(t1);
        
        beq0 = sig(k_sig,1:5,1)';
        beq1 = sig(k_sig,1:5,end)';
        
        Aeq_fr = zeros(N_fr,N_dr);
        beq_fr = zeros(N_fr,1);
        for k_fr = 1:N_fr
            Aeq_fr(k_fr,:) = Ad00(t_sig(1,k_fr+1));
            beq_fr(k_fr,:) = sig(k_sig,1,k_fr+1)';
        end
    end
    
    Aeq = [Aeq0 ; Aeq1 ; Aeq_fr];
    beq = [beq0 ; beq1 ; beq_fr];
    
    % Solve the QP
    c = quadprog(H,[],[],[],Aeq,beq);

    % Roll the solution forward in time to geneate flat outputs
    for k_roll = 1:N_dt
        t_now = k_roll.*fmu_dt;
        f_out(k_sig,:,k_roll) = (Ad04(t_now)*c)';
    end
end

end
