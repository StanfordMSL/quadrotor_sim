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

ndr = model.misc.ndr;
N    = size(obj.kf.fo,3);

% Convert objectives to flat outputs
f_wp.t = obj.kf.t;
f_wp.sigma = zeros(4,ndr,N);
f_wp.sigma(:,1:2,:) = obj.kf.fo;

% Solve the Piecewise QP
f_out = piecewise_QP(f_wp,model.clock.dt_fmu);

% Update the total trajectory
traj = fout2traj(traj,1,f_out,model,mode);

end

