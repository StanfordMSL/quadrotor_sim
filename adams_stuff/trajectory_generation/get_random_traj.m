function [yukf, initial_bb, camera, wp] = get_random_traj(tf)
    global flight
    %%% Initialize YUKF-related stuff %%%
    num_dims = 13; % length of state
    flight.x_act = zeros(num_dims, 1);  % this is a placeholder that needs to happen before yolo_yukf_init()
    yukf = yolo_ukf_init(num_dims, NaN); % this sets most of the filter parameters, the rest are loaded from a file
    
    run_dir = sprintf('adams_stuff/preprocessed_data/run%d', 4);
    yukf.hdwr_prms = read_scenario_params(run_dir, 4);
    data_dir = sprintf('%s/results_%s', run_dir, yukf.hdwr_prms.datetime_str);
    
    initial_bb = init_quad_bounding_box(yukf.hdwr_prms.bb_l, yukf.hdwr_prms.bb_w, yukf.hdwr_prms.bb_h, yukf.hdwr_prms.bb_mult);
    camera = init_camera(yukf);
    
    
    wp_dt = 0.5; % amount of time to go to wp
    
    wp.x_lim = [-4.5, 5];
    wp.y_lim = [-3, 3];
    wp.z_lim = [-1, 4];
    
    base_gate = [ 0.00  0.00  0.00  0.00  0.00;...
             -0.30 -0.30  0.30  0.30 -0.30;...
             -0.15  0.15  0.15 -0.15 -0.15];
    map = [0 0 0]';
    t_now = 0;
    num_wps = ceil(tf / wp_dt);
    N_wp = num_wps;
    t = linspace(0, tf, num_wps + 1); % +1 for the initial 
    x = zeros(12, num_wps + 1);
    x(1, 1) = wp.x_lim(1) + rand*(wp.x_lim(2) - wp.x_lim(1));
    x(2, 1) = wp.y_lim(1) + rand*(wp.y_lim(2) - wp.y_lim(1));
    x(3, 1) = wp.z_lim(1) + rand*(wp.z_lim(2) - wp.z_lim(1));
    while ~is_point_in_view(x(1, 1), x(2, 1), x(3, 1), camera, yukf)
        x(1, 1) = wp.x_lim(1) + rand*(wp.x_lim(2) - wp.x_lim(1));
        x(2, 1) = wp.y_lim(1) + rand*(wp.y_lim(2) - wp.y_lim(1));
        x(3, 1) = wp.z_lim(1) + rand*(wp.z_lim(2) - wp.z_lim(1));
    end
    delta_x = 1;
    delta_y = 1;
    delta_z = 1;
    last_ind = 1;
    
    bias_buffer = 1.5;
    for wind = 2:num_wps
        disp('');
        center_bias_x = 0.2 * sign(x(1, last_ind)) * ...
            double(wp.x_lim(2) - x(1, last_ind) < bias_buffer || x(1, last_ind) - wp.x_lim(1) < bias_buffer);
        center_bias_y = 0.2 * sign(x(2, last_ind)) * ...
            double(wp.y_lim(2) - x(2, last_ind) < bias_buffer || x(2, last_ind) - wp.y_lim(1) < bias_buffer);
        center_bias_z = 0.2 * sign(x(3, last_ind)) * ...
            double(wp.z_lim(2) - x(3, last_ind) < bias_buffer || x(3, last_ind) - wp.z_lim(1) < bias_buffer);
        wp_x = x(1, last_ind) + 2*delta_x*(rand - 0.5 - center_bias_x);
        wp_y = x(2, last_ind) + 2*delta_y*(rand - 0.5 - center_bias_y);
        wp_z = x(3, last_ind) + 2*delta_z*(rand - 0.5 - center_bias_z);
        
        while ~is_point_in_view(wp_x, wp_y, wp_z, camera, yukf)
            wp_x = x(1, last_ind) + 2*delta_x*(rand - 0.5 - center_bias_x);
            wp_y = x(2, last_ind) + 2*delta_y*(rand - 0.5 - center_bias_y);
            wp_z = x(3, last_ind) + 2*delta_z*(rand - 0.5 - center_bias_z);
        end
        
        rand_roll = max(min(pi/4, randn*30*pi/180), -pi/4);
        rand_pitch = max(min(pi/4, randn*30*pi/180), -pi/4);
        
        x(1:3, wind) = [wp_x; wp_y; wp_z];
        quat = eul2quat([rand_roll, rand_pitch, 0], 'XYZ');
        x(7:9, wind) = quat(2:4)';
        last_ind = wind;
    end
    wp.gate = base_gate;
    wp.t = t;
    wp.x = x;
    wp.N_wp = num_wps;
    wp.tf = tf;
    wp.Q_key = 6*ones(num_wps, 2);
    wp.map = map;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end