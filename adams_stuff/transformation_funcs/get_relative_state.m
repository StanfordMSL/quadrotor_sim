function rel_state = get_relative_state(quad_state, cam_state)
    % Terminology:
    %   p_w: point in the world frame [ <var>_<frame> ]
    %   R_c is the SO3 rotation matrix s.t. p_c = R_c_w * p_w
    %   t_c__w_c is a translation vector in frame c pointing to w from c
    %   tf_c_w is the SE3 transformation matrix s.t. p_c = tf_c_w * p_w
    %       tf_c_w = [R_c_w, t_c__w_c; 0, 0, 0, 1] [tf_<frame>__<to frame>_<from frame>]
    %
    %
    %   relative state (12x1): 
    %       t_c__q_c: quad position relative to camera in camera frame (3x1)
    %       v_c__q: quad velocity in the camera frame (3x1)
    %       q_w__q_w: vector elements of quaternion orienting quad in world frame (3x1)
    %       w_q__q: angular velocities of quad in quad's local frame (3x1)
    
    rel_state = zeros(12, 1);
    
    % unpack quad state
    t_w__q_w = quad_state(1:3, 1);
    vel_w__q = quad_state(4:6, 1);
    quat_w__q_w = complete_unit_quat(quad_state(7:9, 1));
    w_w__q = quad_state(10:12, 1);
    
    % unpack camera state
    t_w__c_w = cam_state(1:3, 1);
    vel_w__c = cam_state(4:6, 1);
    quat_w__c_w = complete_unit_quat(cam_state(7:9, 1));
    w_w__c = cam_state(10:12, 1);
    
    
    
    R_w_quad = quat2rotm(quat_w__q_w(:)'); % I confirmed this
    tf_w_quad = [R_w_quad, t_w__q_w(:); [zeros(1, 3), 1]];
    
    R_w_cam = quat2rotm(quat_w__c_w(:)'); 
    R_cam_w = R_w_cam;
    tf_w_cam = [R_w_cam, t_w__c_w(:); [zeros(1, 3), 1]];
    
    tf_quad_cam = inv_tf(tf_w_quad) * tf_w_cam; % seems to be correct with basic tests
    tf_cam_quad = inv_tf(tf_w_cam) * tf_w_quad; % seems to be correct with basic tests
    
    
    t_c__q_c = tf_cam_quad(1:3, 4);
    R_c_q = tf_cam_quad(1:3, 1:3);
    rel_state(1:3) = t_c__q_c;
    rel_state(4:6) = R_cam_w * (vel_w__q - vel_w__c);
    quat_c__q_c = rotm2quat(R_c_q);
    rel_state(7:9) = quat_c__q_c(2:4);
    rel_state(10:12) = R_cam_w * (w_w__q - w_w__c);
    disp();
end