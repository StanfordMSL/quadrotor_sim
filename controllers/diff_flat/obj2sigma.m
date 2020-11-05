function [kf_sig, sigma] = obj2sigma(obj,n_der,N_wp,N_tr)

p_wp = obj.wp_arr(1:3,:);
psi_wp = zeros(1,size(p_wp,2));
sigma_wp = [p_wp ; psi_wp];

if norm(obj.p_g) < 999
    % We have a gate. Need to inject it into the array.
    N_obj = N_wp + 1;
    
    p_gate = obj.p_g;
    psi_gate = 0;
    
    sigma_gate = [p_gate ; psi_gate];
    
%     p12 = obj.p_gc(:,2) - obj.p_gc(:,1);
%     p23 = obj.p_gc(:,3) - obj.p_gc(:,2);
%     sigma_gate_a = cross(p23,p12);
%     sigma_gate_a = sigma_gate_a./norm(sigma_gate_a);
    sigma_gate_a = obj.p_gc(:,2) - obj.p_gc(:,1);
    sigma_gate_a = (12.753./norm(sigma_gate_a)).*sigma_gate_a;
else
    % We don't have a gate. Carry on.
    N_obj = N_wp;
    
    sigma_gate = [];
end

sigma_init = [sigma_wp(:,1) sigma_gate sigma_wp(:,end)];

% Pack it for output
s_wp = zeros(1,N_obj);
for k = 2:N_obj
    s_wp(1,k) = norm(sigma_init(1:3,k)-sigma_init(1:3,k-1)); 
end
s_total = sum(s_wp);

kf_sig = ones(1,N_obj);
for k = 2:N_obj
    fr_seg = round(N_tr*(sum(s_wp(1,1:k))/s_total));
    kf_sig(1,k) = fr_seg; 
end

sigma = zeros(4,n_der,N_obj);
for k = 1:N_obj
    sigma(:,1,k) = sigma_init(:,k); 
end

if norm(obj.p_g) < 999
    for k = 1:3
         sigma(k,3,2) = sigma_gate_a(k,1);
    end
end
