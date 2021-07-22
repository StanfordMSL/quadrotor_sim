function traj = diff_flat(obj,model,traj,mode)

% Solve the trajectory as a sequence of keyframe pairs with n gates in
% between. Note that most trajectories can be built from such a primitive.
% There probably are some exceptions but we can look into this later on.

% The next important thing is that with our trajectory planner, we have a 
% fixed horizon of 10s. This hopefully gives some uniformity to solve times
% in the direct method, assuming a close enough warm start. On top of that,
% we want the warm start to prioritize flying at $(vel). So, we will also
% have to pad the trajectory to hover for the remainder of the 10s.

%% Generate the main part of the trajectory
N_kf = size(obj.kf.x,2)-1;
n_tr = 1;
for k_kf = 1:N_kf
    % Convert objectives to flat outputs
    f_wp = obj2fwp(obj,k_kf,model.df);

    % Solve the Piecewise QP
    f_out = piecewise_QP(f_wp,model.clock.dt_fmu);
    
    % Update the total trajectory
    [traj,n_tr] = fout2traj(traj,n_tr,f_out,model,mode);
end

%% Pad the remainder of the trajectory
traj.T = n_tr;        % Trajectory ends here.

f_end.t = [0 traj.t_fmu(end)-f_wp.t(end)];
f_end.sigma = zeros(4,15,2);
f_end.sigma(:,:,1) = f_wp.sigma(:,:,end);
f_end.sigma(:,:,2) = f_wp.sigma(:,:,end);
f_end.sigma(:,2:5,2) = 0;

% Solve the Piecewise QP
f_out = piecewise_QP(f_end,model.clock.dt_fmu);

% Update the total trajectory
[traj,~] = fout2traj(traj,n_tr,f_out,model,mode);

