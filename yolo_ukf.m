function [sv,mu_prev] = yolo_ukf(sv,flight,k_est,curr_time,initial_bb,camera,qtmp,qm,ukf_prms,mu_curr,mu_prev,sig_curr,model)

% Unpack
yolo_output = predict_quad_bounding_box(flight.x_act(:,k_est), camera, initial_bb); % Add noise??
        
u_est = [];
%         u_est = curr_m_cmd;
[mu_out, sigma_out] = yukf_step(mu_curr, sig_curr, [], yolo_output, model, camera, initial_bb, ukf_prms, mu_prev);
mu_prev = mu_curr;
%         mu_curr = mu_out;
%         sig_curr = sigma_out;

% Save values for plotting %%%%%%%%%%%%%%%%%%
sv.mu_hist(:, k_est) = mu_out;
sv.sig_hist(:, :, k_est) = sigma_out;
sv.sig_trace_hist(k_est) = trace(sigma_out);
sv.time_hist(k_est) = curr_time;
sv.hist_mask(k_est) = true;

qtmp = mu_out(7:9, 1); 
qm = [sqrt(1 - qtmp'*qtmp); qtmp];
[yaw, pitch, roll] = quat2angle(qm(:)');
sv.ypr_hist(:, k_est) = [yaw; pitch; roll];

qtmp = flight.x_act(7:9,k_est); 
qa = [sqrt(1 - qtmp'*qtmp); qtmp];
[yaw, pitch, roll] = quat2angle(qa(:)');
sv.ypr_act_hist(:, k_est) = [yaw; pitch; roll];

sv.ang_err(k_est) = quat_dist(qa, qm);
sv.ang(k_est) = 2*acos(qm(1));
sv.ang_act(k_est) = 2*acos(qa(1));
disp('')