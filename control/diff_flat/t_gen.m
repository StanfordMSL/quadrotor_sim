function t_sigma = t_gen(sigma,misc)

N_wp = size(sigma,3);
t_sigma = zeros(1,N_wp);

% Generate t_sigma
for k_obj = 2:N_wp
    s_int = norm(sigma(1:3,1,k_obj) - sigma(1:3,1,k_obj-1));
    if s_int == 0
        t_int = misc.t_hov;      % to catch the hover case
    else
        t_int = round(s_int/misc.v_cr,1);
    end
    t_sigma(1,k_obj) = t_sigma(1,k_obj-1) + t_int;
end