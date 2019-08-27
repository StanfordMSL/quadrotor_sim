function quat = complete_unit_quat(quat_vec)
    % given the vector component, return the length 4 unit quaterion (scalar first)
    
    if(length(quat_vec) ~= 3)
        error("partial quaternion must be vector component of length 4");
    elseif(norm(quat_vec) > 1)
        error("norm of vector component is > 1 --> invalid unit quaternion (%.3f, %.3f, %.3f)", quat_vec(1), quat_vec(2), quat_vec(3));
    end
    
    % currently quat is a column vector
    quat = [real(sqrt(1 - quat_vec(:)' * quat_vec(:))); quat_vec(:)];
    
    % return quat as row or col vector depending on how it came in
    if(size(quat_vec, 1) < size(quat_vec, 1))
        % make it a row vec
        quat = quat(:)';
    end
end