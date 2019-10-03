function tf_w_quad = state_to_tf(state)
    % returns tf_w_quad given the state vector
    pos_w = state(1:3, 1);
    quat = state(7:10, 1);
    R_w_quad = quat2rotm(quat(:)'); % I confirmed this is the right function
    tf_w_quad = [R_w_quad, pos_w(:); [zeros(1, 3), 1]];
end