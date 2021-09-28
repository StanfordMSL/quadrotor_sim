function [c, cx, cu] = conx_calc(obj_type,x,p_box,map,cone_coeffs,n_x,n_u,N)

% Initialize Variables
c  = zeros(22,N);
cx = zeros(22,n_x,N);
cu = zeros(22,n_u,N);

if (size(p_box,3) > 0)
    p12 = p_box(:,2)-p_box(:,1);
    p14 = p_box(:,4)-p_box(:,1);
    plane = [ cross(p12,p14) ; dot(cross(p12,p14),p_box(:,1))];

    for k = 1:N
        x_k = x(:,k);

        dist = plane_dist(plane,x_k(1:3));
        if dist < 0.1
            % gate constraints
            c(1:16,k)    = conx_gate(x_k,p_box);   
            cx(1:16,:,k) = conx_gate_x(x_k,p_box);
            if obj_type == "grasp"
                % velocity constraints
                c(23,k)    = conx_contact(x_k,cone_coeffs);
                cx(23,:,k) = conx_contact_x(x_k,cone_coeffs);
                cu(23,:,k) = 0;
            end
        end
        % room boundary constraints
        c(17:22,k) = conx_map(x_k,map);
        cx(17:22,:,k) = conx_map_x(x_k,map);
    end
end


end