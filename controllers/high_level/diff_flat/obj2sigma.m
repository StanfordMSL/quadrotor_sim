function [t_sigma, sigma, con_sigma] = obj2sigma(obj,map,t_obj,model,n_der)

% Before computing sigmas, we first need to add waypoints that represent
% obstacles. We shall use a constant velocity assumption.
if any(ismember(fields(map),'p_gc')) == 1
    gate_con = gate_wp_builder(obj,map,model.Ft_max,model.m_est);
    % Total Waypoints (gate included).
    N_wp = size(gate_con,2) + size(obj.x,2);
else
    gate_con = zeros(8,1);
    gate_con(1,1) = 999.0;
    
    N_wp = size(obj.x,2);
end

% Initialize Output Variables
% | 0 for terminal wps | 1 for regular wps | 2 for gates
t_sigma    = zeros(1,N_wp);
sigma      = zeros(4,n_der,N_wp);
con_sigma  = zeros(4,5,N_wp);

k_obj  = 1;
k_gate = 1;
for k_wp = 1:N_wp
    if k_wp == gate_con(end,k_gate)
        % Enforce Gate Constraint
        t_sigma(1,k_wp) = gate_con(1,k_gate);

        sigma(1:3,1,k_wp) = gate_con(2:4,k_gate);
        sigma(1:3,3,k_wp) = gate_con(5:7,k_gate);

        % Setup Constraint Triggers
        con_sigma(:,:,k_wp) = [ 1 0 1 0 0;       % gate wp
                                1 0 1 0 0;
                                1 0 1 0 0;
                                1 0 0 0 0];
        
        % Load next gate index.
        if k_gate < size(gate_con,2)
            k_gate  = k_gate + 1;
        end
    else
        % Enforce Position Constraint
        t_sigma(1,k_wp) = t_obj(1,k_obj);
        
        sigma(1:3,1,k_wp) = obj.x(1:3,k_obj);
        sigma(1:3,2,k_wp) = obj.x(4:6,k_obj);        
        
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


end
