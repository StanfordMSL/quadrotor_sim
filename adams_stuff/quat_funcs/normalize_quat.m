function quat = normalize_quat(quat_vec)
    % normalize quaternion but modifying scalar component
    
    if length(quat_vec) == 3
        quat = complete_unit_quat(quat_vec);
    else
        if( norm(quat_vec) <= 1 )
            sn = sign(quat_vec(1));
            quat = complete_unit_quat(quat_vec(2:4));
            quat(1) = sn * quat(1); % maintain the same sign on first element!   
        elseif( norm(quat_vec) <= 1.001 )
            quat = quat_vec / norm(quat_vec); % no warning needed - close enough
        else
            warning("invalid quaternion - crude normalization")
            quat = quat_vec / norm(quat_vec);
        end
    end
    
    % return quat as row or col vector depending on how it came in
    if(size(quat_vec, 1) < size(quat_vec, 2))
        % make it a row vec
        quat = quat(:)';
    end
end