function predicted_obs = predict_obs(sps, camera, initial_bb, ukf_prms)
    num_sps = size(sps, 2);
    
    % line 7
    predicted_obs = zeros(ukf_prms.meas_len, num_sps);
    for i = 1:num_sps
        predicted_obs(:, i) = predict_quad_bounding_box(sps(:, i), camera, initial_bb);
    end
    
    % line 8
end