function traj = msl_ilqr_ws(obj,wts,model,N_seg,nom_show)

% Initialize the traj struct
traj = msl_lqr_init(N_seg,obj,model);

% Warm start with an initial run
traj = msl_lqr(1,traj,obj,wts,model,'offline');

% Publish some diagnostics
switch nom_show
    case 'show'
        nominal_plot(traj.x_bar,obj,20,'persp');
        motor_debug(traj.u_bar,model)
    case 'hide'
end





% disp(['[warm_start]: Total frame count per segment fixed at ',num2str(N_seg)]);
% TO DO... do a creeping search for sequenced waypoints. we'll need
% some kind of tolerance check to trigger subsequent waypoints

