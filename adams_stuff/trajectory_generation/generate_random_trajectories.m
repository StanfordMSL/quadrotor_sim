function generate_random_trajectories()
    clear; clc; close all; rng(42); warning('');
    addpath(genpath(pwd));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Time and Simulation Rate
    downsample_rate = 30; % hz
    tf = 15;

    est_hz = 100;       % State Estimator Time Counter
    lqr_hz = 1;        % Low Rate Controller Sample Rate
    con_hz = 200;       % High Rate Controller Sample Rate
    act_hz = 1000;      % Actual Dynamics Sample Rate

    sim_dt = 1/lcm(lcm(est_hz,con_hz),lcm(lqr_hz,act_hz));
    sim_N  = tf/sim_dt;

    t_est = 0:1/est_hz:tf;
    t_lqr = 0:1/lqr_hz:tf;
    t_con = 0:1/con_hz:tf;
    t_act = 0:1/act_hz:tf; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Initialize Simulation
    
    %%% Map, Dynamics and Control Initialization
    model  = model_init('simple vII',est_hz,con_hz,act_hz); % Initialize Physics Model
    fc     = fc_init(model,'ilqr');                         % Initialize Controller
    FT_ext = nudge_init(act_hz,tf,'off');                   % Initialize External Forces
    tol = 1e-5;         % Tolerance to trigger various processes
    
    timestamp = datestr(now, 'yyyy_mm_dd_HH_MM_SS');
    my_dir = sprintf('adams_stuff/trajectory_data/%s', timestamp);
    pose_dir = sprintf('%s/poses', my_dir);
    bb_dir = sprintf('%s/bb', my_dir);
    if(exist(pose_dir, 'dir')~=7); mkdir(pose_dir); end
    if(exist(bb_dir, 'dir')~=7); mkdir(bb_dir); end
    
    num_traj = 3;
    bb_cell_arr = cell(num_traj, 1);
    traj_cell_arr = cell(num_traj, 1);
    t_cell_arr = cell(num_traj, 1);
    bb_rc_list_cell_arr = cell(num_traj, 1);
    traj_successful = true(num_traj, 1);
    flight_cell_arr = cell(num_traj, 1);
    wp_cell_arr = cell(num_traj, 1);
    traj_count = 1;
    for tind = 1:num_traj
        
        [yukf, initial_bb, camera, wp] = get_random_traj(tf);
%         plot_traj(wp);
        num_dims = length(yukf.mu);
        flight = flight_init(model,tf,wp);                      % Initialize Flight Variables
        
        %%% Time Counters Initialization
        k_est = 1;          % State Estimator Time Counter
        k_lqr = 1;          % Low Rate Controller Time Counter
        k_con = 1;          % High Rate Controller Time Counter
        k_act = 1;          % Actual Dynamics Time Counter
        k_wp  = 1;          % Waypoint Time Counter
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Cold Start the nominal trajectory for the iLQR
        nom = ilqr_init(flight.t_act(:,1),flight.x_act(:,1),wp,fc,model);
        fprintf('[TRAJ %d]: Warm start complete! Ready to launch! ------------------------\n', tind);
        bb_arr = zeros(yukf.prms.meas_len, sim_N);
        traj_arr = zeros(num_dims, sim_N);
        t_arr = zeros(1, sim_N);
        bb_rc_list = zeros(8, 2, sim_N);
        for k = 1:sim_N
            sim_time = (k-1)*sim_dt;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % State Estimator
            if (abs(t_est(k_est)-sim_time) < tol) && (k_est <= tf*est_hz)
                % Perfect Sensing (used for flight control)
                t_now = t_est(k_est);
                x_now = flight.x_act(:,k_act);
                flight.x_fc(:,k_est) = x_now;
                yukf_state = [x_now(1:6); complete_unit_quat(x_now(7:9)); x_now(10:end)];
                [bb_arr(:, k_est), bb_rc_list(:, :, k_est)] = predict_quad_bounding_box(yukf_state, camera, initial_bb, yukf); %"perfect" prediction
                traj_arr(:, k_est) = yukf_state;
                t_arr(k_est) = t_now;
                
                k_est = k_est + 1;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Low Rate Controller    
            if (abs(t_lqr(k_lqr)-sim_time) < tol) && (k_lqr <= tf*lqr_hz)
                % Update LQR params
                nom = ilqr(t_now,x_now,wp,nom,fc,model);
                k_lqr = k_lqr + 1;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % High Rate Controller    
            if (abs(t_con(k_con)-sim_time) < tol) && (k_con <= tf*con_hz)
                % Draw Out Motor Commands from u_bar computed by iLQR
                del_x = x_now-nom.x_bar(:,k_con);
                del_u = nom.alpha*nom.l(:,:,k_con) + nom.L(:,:,k_con)*del_x;
                u  = nom.u_bar(:,k_con) + del_u;
                curr_m_cmd = wrench2m_controller(u,model);

                % Log State Estimation and Control
                flight.m_cmd(:,k_con) = curr_m_cmd;

                k_con = k_con + 1;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Dynamic Model
            if (abs(t_act(k_act)-sim_time) < tol)
                flight.x_act(:,k_act+1) = quadcopter(flight.x_act(:,k_act),curr_m_cmd,model,FT_ext(:,k_act),'actual');
                k_act = k_act + 1;
            end
            [warnMsg, warnId] = lastwarn;
            if ~isempty(warnMsg) && strcmp('MATLAB:illConditionedMatrix', warnId)
                traj_successful(tind) = false;
                warning('');
                break
            end
            disp('');
        end
        if traj_successful(tind)
            next_t = 0;
            ind = 0;
            fn_pose = sprintf("%s/poses_t%d.txt", pose_dir, traj_count);
            fn_bb = sprintf("%s/bb_t%d.txt", bb_dir, traj_count);
            fh_pose = fopen(fn_pose, 'w');
            fh_bb = fopen(fn_bb, 'w');
            if traj_count == 1
                cleanupObj = onCleanup(@()fclose(fh_pose));
                cleanupObj2 = onCleanup(@()fclose(fh_bb));
            end
            for k = 1:length(t_arr)
                t = t_arr(k);
                if t > next_t
                    fprintf(fh_pose, "%d %d %.6f %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", traj_count, ind, t, traj_arr(1, k), traj_arr(2, k), traj_arr(3, k), traj_arr(7, k), traj_arr(8, k), traj_arr(9, k), traj_arr(10, k));
                    bb_box_xy = bb_5val_to_xys(bb_arr(:, k));
                    fprintf(fh_bb, "%d %d %.6f %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", traj_count, ind, t, bb_box_xy(1, 1), bb_box_xy(1, 2), bb_box_xy(2, 1), bb_box_xy(2, 2), bb_box_xy(3, 1), bb_box_xy(3, 2), bb_box_xy(4, 1), bb_box_xy(4, 2));
                    ind = ind + 1;
                    next_t = next_t + 1/downsample_rate;
                end
            end
            bb_cell_arr{tind} = bb_arr;
            traj_cell_arr{tind} = traj_arr;
            t_cell_arr{tind} = t_arr;
            bb_rc_list_cell_arr{tind} = bb_rc_list;
            flight_cell_arr{tind} = flight;
            wp_cell_arr{tind} = wp;
            traj_count = traj_count + 1;
        end
        fprintf('----- END TRAJECTORY ------------------------\n');
        disp('')
    end
    fclose(fh_pose);
    fclose(fh_bb);
    tn = 1;
    disp('');
    animation_plot_and_bbs(flight_cell_arr{tn}, wp_cell_arr{tn}, traj_cell_arr{tn}, bb_cell_arr{tn}, t_cell_arr{tn}, bb_rc_list_cell_arr{tn}(:, :, :), camera, 'persp');
    disp('')
end