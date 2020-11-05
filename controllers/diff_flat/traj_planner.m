function f_out = traj_planner(kf_sig,sigma,fmu_dt,N_tr,n_der)

% Unpack some stuff
N_wp    = size(sigma,3);

% Initialize flat outputs holder
f_out_full = zeros(4,n_der,N_tr);
f_out_full(:,:,1) = sigma(:,:,1);

for k = 1:(N_wp-1)
    n_dt = kf_sig(1,k+1) - kf_sig(1,k) + 1;     % timespan of trajectory component
    idx1 = kf_sig(k);
    idx2 = kf_sig(k+1);
    
    sig_t0 = f_out_full(:,:,idx1);
    sig_tf = sigma(:,:,k+1);
    if k == 1 && N_wp > 2
        g_flag = 1;
    else
        g_flag = 0;
    end
    
    f_out_full(:,:,idx1:idx2) = pcws_QP(sig_t0,sig_tf,fmu_dt,n_dt,n_der,g_flag);
end

f_out = f_out_full(:,1:5,:);

end