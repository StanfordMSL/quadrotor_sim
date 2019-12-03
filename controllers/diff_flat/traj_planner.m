function [t_out,f_out] = traj_planner(sigma,hz,n_p)

count = sigma.t(end)*hz+1;
f_out = zeros(4,5,count);
k0 = 1;
for k = 1:sigma.kf_k-1
    tf = sigma.t(1,k+1) - sigma.t(1,k);
    kf = k0 + tf*hz;
    
    sig_set = zeros(4,n_p,2);
    if k == 1
        sig_set(:,:,1) = sigma.kf(:,:,k);
    else
        sig_set(:,:,1) = f_out_raw(:,:,end);
    end
    sig_set(:,:,2) = sigma.kf(:,:,k+1);
    
    f_out_raw = piecewise_fout(tf,sig_set,hz,n_p);
    f_out(:,:,k0:kf) = f_out_raw(:,1:5,:);
    k0 = kf+1;
end

t_out = linspace(0,sigma.t(end),count);
