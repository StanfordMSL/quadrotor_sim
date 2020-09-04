function f_out = traj_planner(sigma,fmu_dt,N_tr,n_p)

% Unpack some stuff
N_wp    = size(sigma,3);

% Initialize flat outputs holder
f_out = zeros(4,5,N_tr);

% Generate time allotment for each segment. We conservatively assume gate
% is at midpoint.
midpoint = round(N_tr/2);
N_dt = [1 ; midpoint ; N_tr];

for k = 1:(N_wp-1)
    n_dt  = N_dt(k+1,1) - N_dt(k,1) + 1;     % timespan of trajectory component

    sig_set = zeros(4,n_p,2);
    if k == 1
        sig_set(:,:,1) = sigma(:,:,1);
    else
        sig_set(:,:,1) = f_out_raw(:,:,end);
    end
    sig_set(:,:,2) =  sigma(:,:,k+1);
    
    f_out_raw = piecewise_fout(sig_set,fmu_dt,n_dt,n_p);
    f_out(:,:,N_dt(k):N_dt(k+1)) = f_out_raw(:,1:5,:);
end