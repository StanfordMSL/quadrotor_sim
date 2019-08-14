function axang = quat_to_axang(quat)
    % input is a 4 element quat, output is 3 element vector in axis-angle format
    
    if(abs(quat(1)) > 1)
        if(abs(norm(quat) - 1) < 0.0001)
            % then this is just a numeric issue... also, no rotation!
            axang = [0;0;0];
            return
        else
            error("invalid quaternion!")
        end
    end
    ang = 2 * acos(quat(1));
    
    if(abs(ang) < 0.001)
        % no rotation
        axang = [0;0;0];
        return
    end
    vec = quat(2:4) / norm(quat(2:4));
    axang = ang * vec; 
end