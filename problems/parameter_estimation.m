addpath(genpath(pwd));
clear; clc; 

%% Initialize Model, Objective, Constraint and Misc. Parameters

% Generate Model Parameters
model = param_gen('carlito','mismatch');

% Generate Objective
obj  = obj_gen('line');

% Generate Simulation Dynamics Function
quad_gen(model,'simulation')

% Generate Nominal Trajectory
traj_nom = ms_traj_gen(obj,model.est,200);

% Generate Actual Trajectory
traj_act = wr2traj(traj_nom.U,obj,model.act,200);

%% Plot Trajectory
compare_plot(traj_nom,traj_act,obj,'show',100)

%% Generate Cost Function

% Brute Force
out_brut = brute_force(traj_nom);

% Recursive
out_rec = recursive(traj_nom,traj_act,model.est);