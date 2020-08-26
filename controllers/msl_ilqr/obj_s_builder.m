function obj_s = obj_s_builder(x_curr,obj)
    % current waypoint objective
    obj_s.x_star = obj.wp_arr(:,obj.wp_curr);
    
    % current gate objective
    p_b = x_curr(1:3,1);
    p_g = obj.p_gate;
    
    gate_dist = vecnorm(p_g-p_b);
    [~,obj_gate_index] = min(gate_dist);
    
    obj_s.pnts_gate = obj.pnts_gate(:,:,obj_gate_index);
end