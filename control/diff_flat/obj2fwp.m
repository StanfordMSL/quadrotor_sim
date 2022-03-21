function fwp = obj2fwp(obj)

% Unpack/Define Some Stuff
N_wp = 2;
x_kf = obj.x;
       
% Initialize Our Outputs
t_sigma = zeros(1,N_wp);
sigma   = zeros(4,15,N_wp);

% Generate sigma and con_sigma
sigma(:,:,1) = kf2sigma(x_kf(:,1));
sigma(:,:,2) = kf2sigma(x_kf(:,2));



% Generate t_sigma
for k_obj = 2:N_wp
    s_int = norm(sigma(1:3,1,k_obj) - sigma(1:3,1,k_obj-1));
    if s_int == 0
        t_int = 3.0;      % to catch the hover case
    else
        t_int = round(s_int/1.0,1);
    end
    t_sigma(1,k_obj) = t_sigma(1,k_obj-1) + t_int;
end

fwp.t     = t_sigma; 
fwp.sigma = sigma;

end