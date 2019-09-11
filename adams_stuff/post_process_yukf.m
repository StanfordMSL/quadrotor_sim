function post_process_yukf()   
    close all; clear all; clc; format compact; rng(42);
    global flight k yukf
    fprintf("NOTE: consider using quad offset!!\n")
    
    %%% SCENARIO - Choose to specifiy data & camera position/calibration %%%
    scenario = 3; 
    
    run_dir = sprintf('adams_stuff/preprocessed_data/run%d', scenario);
    hdwr_prms = read_scenario_params(run_dir, scenario);
    data_dir = sprintf('%s/results_%s', run_dir, hdwr_prms.datetime_str);
    
    b_use_perfect_bb = false;
    
    conf_thresh = 0.75; % value below which we decide we did not detect the image
    b_animate = true;
    b_view_from_camera_perspective = false; % show animation from point of view of camera
    animation_pause = 0.05; % [seconds] amound of extra time to pause between animation frames
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% READ IN PROCESSED DATA %%%%%%%%%
    [t_pose_arr, t_rbg_arr, z_mat, position_mat, quat_mat, gt_bb] = load_preprocessed_data(data_dir, hdwr_prms.datetime_str, conf_thresh);
    num_img = length(t_pose_arr);
    fprintf("%d images remaining after discarding confidences < %.0f%%\n", num_img, conf_thresh*100);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%% Initialize YUKF %%%
    % initialize flight, a struct needed by the simulator. for now it holds ground truth
    flight.x_act = zeros(12, num_img);
    flight.x_act(1:3, :) = position_mat';
    flight.x_act(7:9, :) = quat_mat(:, 2:4)';
    flight.t_act = t_pose_arr;
    
    [sv, yukf] = yolo_ukf_init(flight, t_rbg_arr);
    yukf.hdwr_prms = hdwr_prms;  % save hardware params in yukf struct
    camera = init_camera(yukf);
    initial_bb = init_quad_bounding_box('large'); % what size bb are we using? (leave blank for small)
%     initial_bb = init_quad_bounding_box();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% plot ground truth vs yolo vs predicted bounding boxes %%%
    compare_yolo_predictions_and_gt(z_mat, gt_bb, position_mat, quat_mat, camera, initial_bb, yukf);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% Init animation plot %%%
    if b_animate
        [traj_est_h, h_persp, h_persp_est] = init_iterative_animation_plot(position_mat(1,:), quat_mat(1,:), camera, position_mat, quat_mat, b_view_from_camera_perspective);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    %%% YUKF %%%%%%%%%
    for k = 1:num_img
        % YOLO UKF %%%%%%
        if(k > 1)
            t_now = t_rbg_arr(k);
            yukf.dt = t_now - t_rbg_arr(k - 1);
            
            if b_use_perfect_bb
                yolo_output = predict_quad_bounding_box(flight.x_act(:, k), camera, initial_bb, yukf); %"perfect" prediction
                % [gt_bb(k, :)'; 0];
            else
                yolo_output = [z_mat(k, :)'; 0]; 
            end
            yukf = yukf_step(yukf, [], yolo_output, [], camera, initial_bb);

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
    plot_ukf_hist(sv, flight);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    disp('')
end