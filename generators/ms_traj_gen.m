function traj = ms_traj_gen(obj,model,hz)

% Generate Flat Waypoints
f_wp = obj2fwp(obj);

% Generate Flat Outputs
f_out = piecewise_QP(f_wp,1/hz);

% Generate Trajectory
traj = fout2traj(f_out,model,obj.x(:,1),hz);

end