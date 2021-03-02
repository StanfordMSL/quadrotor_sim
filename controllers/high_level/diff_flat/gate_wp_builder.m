function gate_con = gate_wp_builder(obj,map,Ft_max,m)
gate_con = zeros(4,0);

count = 1;
for j = 1:size(obj.x,2)-1
    for k = 1:size(map.p_g,2)
        % Gate Parameters
        p_G1 = map.p_gc(:,1,k);
        p_G2 = map.p_gc(:,2,k);
        p_G4 = map.p_gc(:,4,k);

        r_12 = p_G2 - p_G1;
        r_14 = p_G4 - p_G1;

        n_G  = cross(r_14,r_12);

        % Drone Paremters. We assume a constant velocity of 2.0m/s
        l_0 = obj.x(1:3,j);
        l   = (obj.x(1:3,j+1)-obj.x(1:3,j))./2.0;
        
        den = dot(l,n_G);

        if den == 0
            % never hitting.
        else
            alpha_c = dot((p_G1-l_0),n_G)/den;
            u = dot(cross(r_12,-l),(l_0-p_G1))/dot(-l,cross(r_14,r_12));
            v = dot(cross(-l,r_14),(l_0-p_G1))/dot(-l,cross(r_14,r_12));
            
            if (alpha_c <= 1) && (u >= 0) && (u <= 1) && (v >= 0) && (v <= 1)
                t_wp = alpha_c;  
                x_wp = map.p_g(:,k);
                a_wp = (0.5*Ft_max/m).*(r_12/norm(r_12));

                feed = [t_wp ; x_wp ; a_wp ; j+count];
                gate_con = [gate_con feed];
                count = count+1;
            end
        end
    end
end

end