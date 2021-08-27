function fwp = obj2fwp(obj,seq,misc)

% Unpack/Define Some Stuff
idxs = find(obj.gt.seq==seq);
N_gt = size(idxs,2);
N_kf = size(obj.kf.x,2);
N_wp = N_kf + N_gt;

x_kf = obj.kf.x(:,seq:seq+1);

p_g = zeros(3,N_gt);
for k = 1:N_gt
    idx =  idxs(k);
    p_g(:,k) = obj.gt.p_ctr(:,idx);
end
       
% Initialize Our Outputs
t_sigma = zeros(1,N_wp);
sigma   = zeros(4,misc.ndr,N_wp);

% Generate sigma and con_sigma
sigma(:,:,1) = kf2sigma(x_kf(:,1));

sigma(:,:,end) = kf2sigma(x_kf(:,2));

k_g = 1;
for k_obj = 2:N_wp-1  
    sigma(1:3,1,k_obj) = p_g(:,k_g);
    k_g = k_g + 1;
end

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

fwp.t     = t_sigma; 
fwp.sigma = sigma;

end
