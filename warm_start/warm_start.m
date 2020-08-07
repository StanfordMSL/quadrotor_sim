function traj = warm_start(obj,wts,N_seg,model,nom_show)

% Initialize iLQR variables
[traj,al] = msl_lqr_init(N_seg,16,obj,model);

% Warm start with an initial run
traj = msl_lqr(traj,al,obj,wts,model);

% Publish some diagnostics
switch nom_show
    case 'show'
        nominal_plot(traj,obj,50,'persp');
    case 'hide'
end

disp(['[warm_start]: Total frame count per segment fixed at ',num2str(N_seg)]);

% TO DO... do a creeping search for sequenced waypoints. we'll need
% some kind of tolerance check to trigger subsequent waypoints

