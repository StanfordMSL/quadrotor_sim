clear

load saves/log_long_slit_101.mat;

[N_traj,obj] = obj_init_qual('long slit');           % Initialize objectives

targ         = targ_init('none');


animation_plot(log,obj,targ,'nice','show');
pause(3);
slow_mo_plot(log,obj,targ,'nice','show');
