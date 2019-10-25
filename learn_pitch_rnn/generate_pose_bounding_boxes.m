close all
clear all
clc

global flight yukf

nb_steps = 1000;

flight.x_act = zeros(13, 1);  % this is a placeholder that needs to happen before yolo_yukf_init()
yukf = yolo_ukf_init(13, NaN); % this sets most of the filter parameters, the rest are loaded from a file
scenario = 4; 
run_dir = sprintf('adams_stuff/preprocessed_data/run%d', scenario);
yukf.hdwr_prms = read_scenario_params(run_dir, scenario);
camera = init_camera(yukf);
b_view_from_camera_perspective = false; 
animation_pause = 0.000005;
states = zeros(13,nb_steps);
states(7,1) = 1;

model = model_init('simple vII',0,0,0);
eul = quat2eul(states(7:10,:)')
a_z = randn(1,nb_steps)*model.m_act*model.g/2 +model.m_act*model.g;
torques = randn(3,nb_steps)*pi/16;
dt = 0.05;



for i = 1:nb_steps-1
    omega = states(11:13,i);
    u = quad_inputs_from_acc(a_z(i),torques(:,i),omega',model) ;
    vel_dot = lin_acc(states(:,i), u, model,[0 0 0]', 0, 'actual');
    omega_dot = ang_acc(u, states(11:13,i), model, [0 0 0]', 'actual');
    states(11:13,i+1) = states(11:13,i) + dt * omega_dot;
    states(4:6,i+1) = states(4:6,i) + dt * vel_dot;
    states(1:3,i) = states(1:3,i) + dt * states(4:6,i);
    
    wx = states(11,i);
    wy = states(12,i);
    wz = states(13,i);

    quat = states(7:10,i);
    % Setup Some Useful Stuff for Pred 
    Omega = [ 0 -wx -wy -wz ;...
             wx   0  wz -wy ;...
             wy -wz   0  wx ;...
             wz  wy -wx   0 ];

    q_hat = quat + 0.5 * Omega * quat * dt;
    
    states(7:10,i+1) = q_hat;

end

[traj_est_h, h_persp, h_persp_est] = init_iterative_animation_plot(states(1:3,1)', states(7:10,1)', camera, states(1:3,:)', states(7:10,:)', b_view_from_camera_perspective);
sv.mu_hist = states;
sv.hist_mask = ones(nb_steps,1)';
for i = 1:nb_steps
    [h_persp, h_persp_est] = update_animation_plot(h_persp, traj_est_h, h_persp_est, states(1:3,i)', states(7:10,i)', sv, i, animation_pause);
end