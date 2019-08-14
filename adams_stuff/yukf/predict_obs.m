function predicted_obs = predict_obs(sps, camera, initial_bb, yukf)
    % line 7 from prob rob
    num_sps = size(sps, 2);
    predicted_obs = zeros(yukf.prms.meas_len, num_sps);
    for i = 1:num_sps
        predicted_obs(:, i) = predict_quad_bounding_box(sps(:, i), camera, initial_bb);
    end
end