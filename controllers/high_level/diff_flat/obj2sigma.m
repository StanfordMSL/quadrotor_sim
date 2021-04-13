function [t_sigma, sigma, con_sigma] = obj2sigma(obj,t_obj,n_der)

% Initialize Output Variables
N_wp = size(obj.x,2);
t_sigma    = zeros(1,N_wp);
sigma      = zeros(4,n_der,N_wp);
con_sigma  = zeros(4,5,N_wp);

k_obj  = 1;
for k_wp = 1:N_wp
    % Enforce Position Constraint
    t_sigma(1,k_wp) = t_obj(1,k_obj);
    
    sigma(1:3,1,k_wp) = obj.x(1:3,k_obj);
    sigma(1:3,2,k_wp) = obj.x(4:6,k_obj);
    
    quat = obj.x(7:10,k_obj)';
    eul = quat2eul(quat);
    
    sigma(4,1,k_wp) = eul(1);
    sigma(4,2,k_wp) = 0;
    
    % Setup Constraint Triggers
    if ((k_wp == 1) || (k_wp == N_wp))
        con_sigma(:,:,k_wp) = [ 1 1 1 1 1;       % terminal wp
            1 1 1 1 1;
            1 1 1 1 1;
            1 1 1 0 0];
    else
        con_sigma(:,:,k_wp) = [ 1 1 1 1 1;       % regular wp
            1 1 1 1 1;
            1 1 1 1 1;
            1 1 1 0 0];
    end
    
    % Load next obj index.
    k_obj  = k_obj + 1;
end

end
