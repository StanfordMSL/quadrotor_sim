function post_process_yukf()   
    close all; clear all; clc; format compact; rng(42); addpath(genpath(pwd));
    global flight k yukf t_tmp
    fprintf("NOTE: consider using quad offset!!\n")
    
    %%% Initialize YUKF %%%
    num_dims = 13; % length of state
    flight.x_act = zeros(num_dims, 1);  % this is a placeholder that needs to happen before yolo_yukf_init()
    yukf = yolo_ukf_init(num_dims, NaN); % this sets most of the filter parameters, the rest are loaded from a file

    %%% SCENARIO - Choose to specifiy data & camera position/calibration %%%
    scenario = 4; 
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% overwrite placeholder value of flight.x_act/t_act with ground truth data %%%
    flight.x_act = zeros(num_dims, num_img); 
    flight.x_act(1:3, :) = position_mat';
    flight.x_act(7:10, :) = quat_mat(:, :)';
    flight.t_act = t_pose_arr;
    x0_gt = [position_mat(1, :)'; zeros(3, 1); quat_mat(1, :)'; zeros(3 ,1)];
    yukf.dt = flight.t_act(2) - flight.t_act(1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if 0
        fn = 'adams_stuff/preprocessed_data/thor_results_tracking.txt';
        fid = fopen(fn,'rt');
        line = textscan(fid,'%s','Delimiter','\n');
        N = min(yukf.hdwr_prms.end_img_ind + 1, size(line{1}, 1));
        thor_data = zeros(N, 4);

        for ii = 1:N
            output = strsplit(line{1}{ii}, ' ');
            frame_ind = str2double(output{1}(5:end));
            thor_data(ii, 1) = str2double(output{3});
            thor_data(ii, 2) = str2double(output{4});
            thor_data(ii, 3) = str2double(output{5});
            thor_data(ii, 4) = str2double(output{6});
            if yukf.hdwr_prms.end_img_ind > 0 && frame_ind > yukf.hdwr_prms.end_img_ind
%                 skipped_outputs(ind:end) = true;
                break;
            end
        end
%         thor_data_trimmed = thor_data;
%         thor_data_trimmed(~kept_frame_ids(1:size(thor_data_trimmed, 1)), :) = [];

        fclose(fid);
        compare_thor(thor_data, gt_bb, position_mat, quat_mat, camera, initial_bb, yukf);
%         compare_yolo_predictions_and_gt(thor_data, gt_bb, position_mat, quat_mat, camera, initial_bb, yukf);
    end
    
    %%% INITALIZE THE FILTER'S ESTIMATE (and variable for saving data) %%%
    if yukf.b_offset_at_t0
        pos0_est = [1; 2; -1] + x0_gt(1);
        v0_est = [0; 0; 0] + x0_gt(4:6);
        yaw0 = 0;   pitch0 = 1*pi/180;   roll0 = 3*pi/180;
        quat0_est = quatmultiply(x0_gt(7:10)', angle2quat(yaw0, pitch0, roll0))';
        w0_est = [0; 0; 0] + x0_gt(11:13);
        yukf.mu = [pos0_est(:); v0_est(:); quat0_est(:); w0_est(:)];
    else
        yukf.mu = x0_gt(:);
    end
    sv = initialize_variable_for_recording_data(x0_gt, yukf.mu(:), yukf, num_dims, length(flight.t_act)); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    %%% Init animation plot %%%
    if b_animate
        [traj_est_h, h_persp, h_persp_est] = init_iterative_animation_plot(position_mat(1,:), quat_mat(1,:), camera, position_mat, quat_mat, b_view_from_camera_perspective);
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
            yukf = yukf_step(yukf, [], yolo_output, [], camera, initial_bb);
            
            if yukf.prms.b_filter_data && ~yukf.prms.b_predicted_bb
                % only filter x/y/z
                mv_counter_index = mod(mv_ave_counter - 1, yukf.prms.mv_ave_len_out) + 1;
                mv_ave_out_buff(mv_counter_index, :) = yukf.mu(1:3)';
                yukf.mu(1:3) = mean(mv_ave_out_buff(1:min(mv_counter_index, yukf.prms.mv_ave_len_out), :), 1)';
                mv_ave_counter = mv_ave_counter + 1;
            end

            % Save values for plotting %%%%%%%%%%%%%%%%%%
            sv = update_save_var(sv, k, yukf, flight, t_now);
        end
        %%%%%%%%%%%%%%%%%%%
        if b_animate
            [h_persp, h_persp_est] = update_animation_plot(h_persp, traj_est_h, h_persp_est, position_mat(k, :), quat_mat(k, :), sv, k, animation_pause);
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