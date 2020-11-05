function f_out_full = pcws_QP(sig_t0,sig_tf,fmu_dt,N_dt,n_der,g_flag)

% Unpack Some Useful Stuff
tf = fmu_dt * (N_dt-1);

% Free Parameters
mu_r   = 1;
mu_psi = 1;
k_r    = 4;
k_psi  = 3;

% Compute the H matrices
H_r = H_build(tf,n_der,k_r);
H_psi = H_build(tf,n_der,k_psi);

% Build the t matrices
t_vect_t0 = t_vect_compute(0,n_der);
t_vect_tf = t_vect_compute(tf,n_der);

% Remaining parameters needed by QP
opt = mpcInteriorPointOptions;
f = zeros(n_der,1);
x0 = ones(n_der,1);
A = zeros(0,n_der);
b = zeros(0,1);

% Initialize Output
f_out_full = zeros(4,n_der,N_dt);

% Solve the QP for each axis.
for j =1:4
    % Choose H based on position or yaw.
    if j == 4
        k_der = k_psi;
        H = mu_psi.*H_psi;
    else
        k_der = k_r;
        H = mu_r.*H_r;
    end

    % t0 Constraints (up to k_r/k_psi th derivative)
    [Aeq_t0,beq_t0] = pcws_eq_builder(k_der,n_der,t_vect_t0,sig_t0(j,:));
    
    % tf Constraints (up to 1st if gate else k_r/k_psi th derivative).
    if g_flag == 1  % it is a gate
        [Aeq_tf,beq_tf] = pcws_eq_builder_v2(2,n_der,t_vect_tf,sig_tf(j,:));
    else            % it is not a gate
        [Aeq_tf,beq_tf] = pcws_eq_builder(k_der,n_der,t_vect_tf,sig_tf(j,:));
    end
  
    Aeq = [Aeq_t0 ; Aeq_tf];
    beq = [beq_t0 ; beq_tf];
        
    [c,~] = mpcInteriorPointSolver(H,f,A,b,Aeq,beq,x0,opt);
    
    for k = 1:N_dt
        t_now = fmu_dt*(k-1);
        t_v_base = t_vect_compute(t_now,n_der);
        
        for m = 1:n_der
            idx1 = m;
            idx2 = n_der-m+1;
            
            c_vect_curr = c(idx1:end,1);
            t_vect_curr = t_v_base(1:idx2,1);
            
            f_out_full(j,m,k) = t_vect_curr' * c_vect_curr;
        end
    end
end


end


    
