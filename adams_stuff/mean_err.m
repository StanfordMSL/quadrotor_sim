function [pos_err, ang_err] = mean_err(ground_truth, yukf_output)
    %  pos_err, ang_err = mean_err(sv.mu_hist, flight.x_act)

    if size(ground_truth, 1) > size(ground_truth, 2)
        ground_truth = ground_truth';
        yukf_output = yukf_output';        
    end
    
    N = min(size(ground_truth, 2), size(yukf_output, 2));
    M = min(size(ground_truth, 1), size(yukf_output, 1));
    pos_err_array = sum((ground_truth(1:3, 1:N) - yukf_output(1:3, 1:N)).^2, 1).^0.5;
    pos_err = mean(pos_err_array);

    ang_err_array = zeros(N, 1);
    if M > 8
        for i = 1:N
            q1 = complete_unit_quat(ground_truth(7:9, i));
            q2 = complete_unit_quat(yukf_output(7:9, i));
            ang_err_array(i) = quat_dist(q1, q2);
        end
        ang_err = mean(ang_err_array);
    else
        ang_err = [];
    end
    figure(342342); clf; grid on;
    subplot(2, 1, 1);
    plot(1:length(pos_err_array), pos_err_array, 'b-')
    ylabel('position error [m]'); xlabel('iteration')
    title(sprintf("Mean Position Error: %.3f m", pos_err))
    subplot(2, 1, 2);
    plot(1:length(ang_err_array), ang_err_array, 'b-')
    ylabel('angle error [deg]'); xlabel('iteration')
    title(sprintf("Mean Angular Error: %.2f def", ang_err))
    disp('')
end