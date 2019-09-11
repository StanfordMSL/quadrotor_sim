function camera = init_camera(yukf)
    % This file defines the camera
    
    % where is the camera
    if ~isfield(yukf.hdwr_prms, 'camera_scenario')
        % default (and sim-only) scenario
        camera_position_w = [-8, 0, 0]; % world frame
        ang_x = 90; ang_y = 0; ang_z = 90;
        Rx = [ 1.  , 0.          ,     0.       ; 0.  , cosd(ang_x) ,-sind(ang_x)  ; 0.  , sind(ang_x) , cosd(ang_x)  ];
        Ry = [cosd(ang_y) , 0.    , sind(ang_y)  ;  0.  ,         1.       , 0            ; -sind(ang_y) , 0.   , cosd(ang_y)  ];
        Rz = [ cosd(ang_z) ,-sind(ang_z) , 0.  ; sind(ang_z) , cosd(ang_z) , 0.  ;  0.              , 0.        1.   ];
        R_cam_w = Rx * Ry * Rz;
        R_w_cam = R_cam_w';
        camera_quat = rotm2quat(R_w_cam);
        camera_pose = [camera_position_w, camera_quat]; % [x,y,z, quat] in world frame 
        camera.quad_offset_w = [0, 0, 0];  

        % store values in struct
        camera.pose = camera_pose;
        camera.tf_w_cam = [R_w_cam, camera_position_w(:); [zeros(1, 3), 1]];
        camera.tf_cam_w = [R_w_cam', -R_w_cam'*camera_position_w(:); [zeros(1, 3), 1]];
    else
        %%% Use hardware params loaded from file
        quat = yukf.hdwr_prms.camera_quat_w(:);
        quat = complete_unit_quat(quat(2:4))';  % normalize quat from optitrack
        tf_w_copt = zeros(4, 4);
        tf_w_copt(1:3, 4) = yukf.hdwr_prms.camera_pos_w(:);
        tf_w_copt(1:3, 1:3) = quat2rotm(quat);

        % this is manual calibration for turning copt frame to camera frame
        tf_cam_copt = tf_from_rpy_and_trans(yukf.hdwr_prms.camera_rot_x, ...
                                            yukf.hdwr_prms.camera_rot_y, ...
                                            yukf.hdwr_prms.camera_rot_z, ...
                                            yukf.hdwr_prms.camera_shift);

        % store values in struct
        camera.tf_w_cam = tf_w_copt * inv_tf(tf_cam_copt);
        camera.tf_cam_w = inv_tf(camera.tf_w_cam);
        camera.pose = [camera.tf_w_cam(1:3, 4)', rotm2quat(camera.tf_w_cam(1:3, 1:3))];
    end
    
    %%% Set Camera intrinsics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    K_image_to_pix =  [617.2744140625, 0.0,              324.1011047363281, 0.;
                       0.0,            617.335693359375, 241.5790557861328, 0.;
                       0.0,            0.0,              1.0              , 0.];
    
    camera.K = K_image_to_pix(1:3, 1:3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % for plotting only
    len = 0.75;
    camera.draw_len = len;
    cam_width = len / K_image_to_pix(1,1) * K_image_to_pix(1, 3)*2;
    cam_height = len / K_image_to_pix(2,2) * K_image_to_pix(2, 3)*2;
    camera.viz_wf = [0, 0, 0; cam_width/2, cam_height/2, len; NaN, NaN, NaN; ...
                     0, 0, 0; -cam_width/2, cam_height/2, len; NaN, NaN, NaN;  ...
                     0, 0, 0; cam_width/2, -cam_height/2, len; NaN, NaN, NaN;  ...
                     0, 0, 0; -cam_width/2, -cam_height/2, len; NaN, NaN, NaN;  ...
                     -cam_width/2, -cam_height/2, len; -cam_width/2, cam_height/2, len; NaN, NaN, NaN;  ...
                     cam_width/2, -cam_height/2, len; cam_width/2, cam_height/2, len; NaN, NaN, NaN;  ...
                     -cam_width/2, -cam_height/2, len; cam_width/2, -cam_height/2, len; NaN, NaN, NaN;  ...
                     -cam_width/2, cam_height/2, len; cam_width/2, cam_height/2, len];
end


%     switch yukf.prms.camera_scenario
%         case 2
%             % these values are taken from Q's EKF code...
%             %
%             % here the camera has a optitrack markers placed on it, but this
%             % frame is not the "camera frame" with z facing forward, etc.
% 
%             % this is what would come out of optitrack:
%             tf_w_copt = [0.99632056,  0.07891092, -0.03344271, -3.16877985; ...
%                         -0.07840041,  0.99678847,  0.01631317, -0.32053804;...
%                          0.0346226 , -0.01363122,  0.99930749,  1.53210664;...
%                          0.        ,  0.        ,  0.        ,  1.        ];
% 
%             % this is manual calibration for turning copt frame to camera frame
%             ang_x = 89.2;
%             ang_y = 90;
%             ang_z = 90;
%             Rx = [ 1.  , 0.          ,     0.       ; 0.  , cosd(ang_x) ,-sind(ang_x)  ; 0.  , sind(ang_x) , cosd(ang_x)  ];
%             Ry = [cosd(ang_y) , 0.    , sind(ang_y)  ;  0.  ,         1.       , 0            ; -sind(ang_y) , 0.   , cosd(ang_y)  ];
%             Rz = [ cosd(ang_z) ,-sind(ang_z) , 0.  ; sind(ang_z) , cosd(ang_z) , 0.  ;  0.              , 0.        1.   ];
%             R_cam_copt = Rx * Rz;
%             t_copt_cam = [0.026, 0.03, -0.019]';
%             tf_cam_copt = zeros(4, 4);
%             tf_cam_copt(1:3, 1:3) = R_cam_copt;
%             tf_cam_copt(1:3, 4) = R_cam_copt' * -t_copt_cam;
%             
%             camera.quad_offset_w = [0, 0, 0.03];
% 
%             % store values in struct
%             camera.tf_w_cam = tf_w_copt * inv_tf(tf_cam_copt);
%             camera.tf_cam_w = inv_tf(camera.tf_w_cam);
%             camera.pose = [camera.tf_w_cam(1:3, 4)', rotm2quat(camera.tf_w_cam(1:3, 1:3))];
%         case 3
%             % these valuse are from the pose_RealSense_2019-03-09-12-14-29.bag
%             pos = [-5.1140, 0.13905, 1.43025];
%             quat = [0.99947, -0.0165, 0.0065, -0.0265];
%             tf_w_copt = zeros(4, 4);
%             tf_w_copt(1:3, 4) = pos';
%             tf_w_copt(1:3, 1:3) = quat2rotm(quat(:)');
% 
%             % this is manual calibration for turning copt frame to camera frame
%             ang_x = 89.2;
%             ang_y = 90;
%             ang_z = 90;
%             Rx = [ 1.  , 0.          ,     0.       ; 0.  , cosd(ang_x) ,-sind(ang_x)  ; 0.  , sind(ang_x) , cosd(ang_x)  ];
%             Ry = [cosd(ang_y) , 0.    , sind(ang_y)  ;  0.  ,         1.       , 0            ; -sind(ang_y) , 0.   , cosd(ang_y)  ];
%             Rz = [ cosd(ang_z) ,-sind(ang_z) , 0.  ; sind(ang_z) , cosd(ang_z) , 0.  ;  0.              , 0.        1.   ];
%             R_cam_copt = Rx * Rz;
%             t_copt_cam = [0.026, 0.03, -0.019]';
%             tf_cam_copt = zeros(4, 4);
%             tf_cam_copt(1:3, 1:3) = R_cam_copt;
%             tf_cam_copt(1:3, 4) = R_cam_copt' * -t_copt_cam;
%             
%             camera.quad_offset_w = [0, 0, 0.03];
% 
%             % store values in struct
%             camera.tf_w_cam = tf_w_copt * inv_tf(tf_cam_copt);
%             camera.tf_cam_w = inv_tf(camera.tf_w_cam);
%             camera.pose = [camera.tf_w_cam(1:3, 4)', rotm2quat(camera.tf_w_cam(1:3, 1:3))];
%         case 4
%             % these valuse are from the pose_RealSense_2019-09-10-12-50-30.bag
%             pos = [-5.441, 0.0072, 1.02531];
%             quat = [-0.998921394348, 0.00403045583516, -0.0443414188921, -0.0131816724315];
%             quat = complete_unit_quat(quat(2:4));  % normalize quat from optitrack
%             tf_w_copt = zeros(4, 4);
%             tf_w_copt(1:3, 4) = pos';
%             tf_w_copt(1:3, 1:3) = quat2rotm(quat(:)');
% 
%             % this is manual calibration for turning copt frame to camera frame
% %             dax = 0; day = 0; daz = 0;
%             dax = 6.2; day = 0; daz = 0.8;
%             ang_x = 90 - dax;%83.8;
%             ang_y = 90 - day; % this value is not used...
%             ang_z = 90 - daz;%89.2;
%             Rx = [ 1.  , 0.          ,     0.       ; 0.  , cosd(ang_x) ,-sind(ang_x)  ; 0.  , sind(ang_x) , cosd(ang_x)  ];
%             Ry = [cosd(ang_y) , 0.    , sind(ang_y)  ;  0.  ,         1.       , 0            ; -sind(ang_y) , 0.   , cosd(ang_y)  ];
%             Rz = [ cosd(ang_z) ,-sind(ang_z) , 0.  ; sind(ang_z) , cosd(ang_z) , 0.  ;  0.              , 0.        1.   ];
%             R_cam_copt = Rx * Rz;
%             t_copt_cam = [0.026, 0.03, -0.019]';
%             tf_cam_copt = zeros(4, 4);
%             tf_cam_copt(1:3, 1:3) = R_cam_copt;
%             tf_cam_copt(1:3, 4) = R_cam_copt' * -t_copt_cam;
%             
%             camera.quad_offset_w = [0, 0, 0.03];
% 
%             % store values in struct
%             camera.tf_w_cam = tf_w_copt * inv_tf(tf_cam_copt);
%             camera.tf_cam_w = inv_tf(camera.tf_w_cam);
%             camera.pose = [camera.tf_w_cam(1:3, 4)', rotm2quat(camera.tf_w_cam(1:3, 1:3))];
%         case 5
%             % these valuse are from the pose_RealSense_2019-09-10-16-58-50.bag
%             % (should be same as that from pose_RealSense_2019-09-10-17-01-41.bag)
%             pos = [-5.67986297607, -0.248037755489, 1.20331776142];
%             quat = [0000, -0.021266086027, 0.02556671947241, -0.0171018615365];
%             quat = complete_unit_quat(quat(2:4));  % normalize quat from optitrack
%             tf_w_copt = zeros(4, 4);
%             tf_w_copt(1:3, 4) = pos';
%             tf_w_copt(1:3, 1:3) = quat2rotm(quat(:)');
% 
%             % this is manual calibration for turning copt frame to camera frame
%             ang_x = 83.8;
%             ang_y = 90; % this value is not used...
%             ang_z = 89.2;
%             Rx = [ 1.  , 0.          ,     0.       ; 0.  , cosd(ang_x) ,-sind(ang_x)  ; 0.  , sind(ang_x) , cosd(ang_x)  ];
%             Ry = [cosd(ang_y) , 0.    , sind(ang_y)  ;  0.  ,         1.       , 0            ; -sind(ang_y) , 0.   , cosd(ang_y)  ];
%             Rz = [ cosd(ang_z) ,-sind(ang_z) , 0.  ; sind(ang_z) , cosd(ang_z) , 0.  ;  0.              , 0.        1.   ];
%             R_cam_copt = Rx * Rz;
%             t_copt_cam = [0.026, 0.03, -0.019]';
%             tf_cam_copt = zeros(4, 4);
%             tf_cam_copt(1:3, 1:3) = R_cam_copt;
%             tf_cam_copt(1:3, 4) = R_cam_copt' * -t_copt_cam;
%             
%             camera.quad_offset_w = [0, 0, 0.03];
% 
%             % store values in struct
%             camera.tf_w_cam = tf_w_copt * inv_tf(tf_cam_copt);
%             camera.tf_cam_w = inv_tf(camera.tf_w_cam);
%             camera.pose = [camera.tf_w_cam(1:3, 4)', rotm2quat(camera.tf_w_cam(1:3, 1:3))];
%         otherwise % INCLUDES CASE 1!!
%             camera_position_w = [-8, 0, 0]; % world frame
%             ang_x = 90;
%             ang_y = 0;
%             ang_z = 90;
%             Rx = [ 1.  , 0.          ,     0.       ; 0.  , cosd(ang_x) ,-sind(ang_x)  ; 0.  , sind(ang_x) , cosd(ang_x)  ];
%             Ry = [cosd(ang_y) , 0.    , sind(ang_y)  ;  0.  ,         1.       , 0            ; -sind(ang_y) , 0.   , cosd(ang_y)  ];
%             Rz = [ cosd(ang_z) ,-sind(ang_z) , 0.  ; sind(ang_z) , cosd(ang_z) , 0.  ;  0.              , 0.        1.   ];
%             R_cam_w = Rx * Ry * Rz;
%             R_w_cam = R_cam_w';
%             camera_quat = rotm2quat(R_w_cam);
%             camera_pose = [camera_position_w, camera_quat]; % [x,y,z, quat] in world frame 
%             
%             camera.quad_offset_w = [0, 0, 0];  
% 
%             % store values in struct
%             camera.pose = camera_pose;
%             camera.tf_w_cam = [R_w_cam, camera_position_w(:); [zeros(1, 3), 1]];
%             camera.tf_cam_w = [R_w_cam', -R_w_cam'*camera_position_w(:); [zeros(1, 3), 1]];
%     end