function post_process_yukf()   
    close all; clear all; clc; format compact; rng(42); addpath(genpath(pwd));
    global flight k yukf t_tmp
    fprintf("NOTE: consider using quad offset!!\n")
    
    %%% Initialize YUKF %%%
    num_dims = 12; % length of state
    flight.x_act = zeros(num_dims, 1);  % this is a placeholder that needs to happen before yolo_yukf_init()
    yukf = yolo_ukf_init(num_dims, NaN); % this sets most of the filter parameters, the rest are loaded from a file
    pf = robotics.ParticleFilter;
    pf.StateEstimationMethod = 'maxweight';
    pf.ResamplingMethod = 'systematic';
    pf.StateTransitionFcn = @stateTransition;
    pf.MeasurementLikelihoodFcn = @measurementLikelihood;
    %%% SCENARIO - Choose to specifiy data & camera position/calibration %%%
    scenario = 3; 
    run_dir = sprintf('adams_stuff/preprocessed_data/run%d', scenario);
    yukf.hdwr_prms = read_scenario_params(run_dir, scenario);
    data_dir = sprintf('%s/results_%s', run_dir, yukf.hdwr_prms.datetime_str);
    
    initial_bb = init_quad_bounding_box(yukf.hdwr_prms.bb_l, yukf.hdwr_prms.bb_w, yukf.hdwr_prms.bb_h, yukf.hdwr_prms.bb_mult);
    
    conf_thresh = 0.5; % value below which we decide we did not detect the image
    b_animate = true;
    b_view_from_camera_perspective = false; % show animation from point of view of camera
    animation_pause = 0.05; % [seconds] amound of extra time to pause between animation frames
    camera = init_camera(yukf);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% READ IN PROCESSED DATA %%%%%%%%%
    [t_pose_arr, t_rbg_arr, z_mat, position_mat, quat_mat, gt_bb] = load_preprocessed_data(data_dir, yukf, conf_thresh);
    num_img = length(t_pose_arr);
    fprintf("%d images remaining after discarding confidences < %.0f%%\n", num_img, conf_thresh*100);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% overwrite placeholder value of flight.x_act/t_act with ground truth data %%%
    flight.x_act = zeros(num_dims, num_img); 
    flight.x_act(1:3, :) = position_mat';
    flight.x_act(7:9, :) = quat_mat(:, 2:4)';
    flight.t_act = t_pose_arr;
    x0_gt = [position_mat(1, :)'; zeros(3, 1); quat_mat(1, 2:4)'; zeros(3 ,1)];
    yukf.dt = flight.t_act(2) - flight.t_act(1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% INITALIZE THE FILTER'S ESTIMATE (and variable for saving data) %%%
    if yukf.b_offset_at_t0
        pos0_est = [1; 2; -1] + x0_gt(1);
        v0_est = [0; 0; 0] + x0_gt(4:6);
        yaw0 = 0;   pitch0 = 1*pi/180;   roll0 = 3*pi/180;
        quat0_est = quatmultiply(complete_unit_quat(x0_gt(7:9))', angle2quat(yaw0, pitch0, roll0))';
        w0_est = [0; 0; 0] + x0_gt(10:12);
        yukf.mu = [pos0_est(:); v0_est(:); quat0_est(2:4); w0_est(:)];
    else
        yukf.mu = x0_gt(:);
    end
    initialize(pf,2000,x0_gt(:),yukf.sigma*0.5);
    sv = initialize_variable_for_recording_data(x0_gt, yukf.mu(:), yukf, num_dims, length(flight.t_act)); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    %%% Init animation plot %%%
    if b_animate
        [traj_est_h, h_persp, h_persp_est] = init_iterative_animation_plot(position_mat(1,:), quat_mat(1,:), camera, position_mat, quat_mat, b_view_from_camera_perspective);
        scatter_plot = scatter3(pf.Particles(:,1),pf.Particles(:,2),pf.Particles(:,3),'.');
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    %%% YUKF %%%%%%%%%
    t_diff = t_rbg_arr(2:end) - t_rbg_arr(1:end-1);
    dt_ave = mean(t_diff);
    mv_ave_in_buff = zeros(yukf.prms.mv_ave_len_in, 2);
    mv_ave_out_buff = zeros(yukf.prms.mv_ave_len_out, 3);
    mv_ave_counter = 1;
    yolo_hist = zeros(yukf.prms.meas_len, num_img);
    for k = 1:num_img
        % YOLO UKF %%%%%%
        if(k > 1)
            t_now = t_rbg_arr(k);
            t_tmp = t_now;
            yukf.dt = t_now - t_rbg_arr(k - 1);
            if yukf.dt > dt_ave*10
                warning('skipped frames!! dt is %.3f seconds (ave dt = %.3f)', yukf.dt, dt_ave);
            end
            
%             yolo_output = querry_sensor(z_mat(k, :), yukf, flight.x_act(:, k), camera, initial_bb);
            if yukf.prms.b_predicted_bb
                yolo_output = predict_quad_bounding_box(flight.x_act(:, k), camera, initial_bb, yukf); %"perfect" prediction
            else
                yolo_output = z_mat(k, :)';
                yolo_output = augment_measurement(yolo_output, yukf, flight.x_act(:, k));
            end
            
            if yukf.prms.b_filter_data && ~yukf.prms.b_predicted_bb
                % only filter width/height of bb
                mv_counter_index = mod(mv_ave_counter - 1, yukf.prms.mv_ave_len_in) + 1;
                mv_ave_in_buff(mv_counter_index, :) = yolo_output(3:4)';
                yolo_output(3:4) = mean(mv_ave_in_buff(1:min(mv_counter_index, yukf.prms.mv_ave_len_in), :), 1)';
                mv_ave_counter = mv_ave_counter + 1;
            end
            
            yolo_hist(:, k) = yolo_output;  % record piltered/augmented yolo output
            %yukf = yukf_step(yukf, [], yolo_output, [], camera, initial_bb);
            [robotPred,robotCov] = predict(pf,[],[],yukf.dt, yukf.prms.Q);
            [robotCorrected,robotCov] = correct(pf,yolo_output,yukf.prms.R,camera, initial_bb,yukf);
            
            if yukf.prms.b_filter_data && ~yukf.prms.b_predicted_bb
                % only filter x/y/z
                mv_counter_index = mod(mv_ave_counter - 1, yukf.prms.mv_ave_len_out) + 1;
                mv_ave_out_buff(mv_counter_index, :) = yukf.mu(1:3)';
                yukf.mu(1:3) = mean(mv_ave_out_buff(1:min(mv_counter_index, yukf.prms.mv_ave_len_out), :), 1)';
                mv_ave_counter = mv_ave_counter + 1;
            end

            % Save values for plotting %%%%%%%%%%%%%%%%%%
            yukf.mu = robotCorrected;
            sv = update_save_var(sv, k, yukf, flight, t_now);
        end
        %%%%%%%%%%%%%%%%%%%
        if b_animate
            [h_persp, h_persp_est] = update_animation_plot(h_persp, traj_est_h, h_persp_est, position_mat(k, :), quat_mat(k, :), sv, k, animation_pause);
            scatter_plot.XData = pf.Particles(:,1);
            scatter_plot.YData = pf.Particles(:,2);
            scatter_plot.ZData = pf.Particles(:,3);
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % plot history of filter vs ground truth
    disp('')
    [prct_crt_1, prct_crt_5, prct_crt_10, err_vec, err_ave, err_min, err_max] = metric1_6dof(sv.mu_hist, sv.mu_act, initial_bb);
    [prct_crt, err_pos_vec, err_ang_vec] = metric2_6dof(sv.mu_hist, sv.mu_act);
    plot_ukf_hist(sv, flight);
   
    
    % plot ground truth vs yolo vs predicted bounding boxes 
    if ~yukf.prms.b_predicted_bb
        compare_yolo_predictions_and_gt(yolo_hist, gt_bb, position_mat, quat_mat, camera, initial_bb, yukf);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    disp('')
end