function [name,T,X,U] = loadtraj(flightID)

dir_base = dir([pwd '/misc/sysID/' flightID '/*.csv']);
name = dir_base.name;
address = [dir_base.folder '/' name];

D = csvread(address);

T = D(1,:);
X = D(2:14,:);
X(1:3,:) = X(1:3,:) - X(1:3,1);     % keep things centered
U = D(15:end,:);

% disp(['[loadtraj]: Trajectory is a ' name ' type.']);
% disp('[loadtraj]: Trajectory centered to start at origin.');