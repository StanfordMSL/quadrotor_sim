function [t_out,f_out] = traj_planner(wp,N,hz,n_p)

f_out = zeros(4,5,N);
k0 = 1;
for k = 1:(wp.N_wp-1)
    t_span  = wp.t(1,k+1) - wp.t(1,k);     % timespan of trajectory component
    fr_span = round(k0 + t_span*hz);              % corresponding frame span.
    
    sig_set = zeros(4,n_p,2);
    if k == 1
        sig_set(:,:,1) = wp.sigma(:,:,1);
    else
        sig_set(:,:,1) = f_out_raw(:,:,end);
    end
    sig_set(:,:,2) = wp.sigma(:,:,k+1);
    
    till = fr_span-k0+1;      % fix to handle unrounded time steps.
    f_out_raw = piecewise_fout(t_span,sig_set,hz,n_p);
    f_out(:,:,k0:fr_span) = f_out_raw(:,1:5,1:till);
    k0 = fr_span;
end

t_out = linspace(0,wp.t(end),N);
