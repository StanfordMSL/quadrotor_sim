function obj = obj_init(trajectory)
     
address = ['trajectories/',trajectory,'.csv'];

% Gate Data
if isfile(address)
    obj.x = readmatrix(address, 'Range', [2 2]);
    
    disp(['[obj_init]: Loaded Trajectory: ',trajectory]);
else
    obj.x = zeros(13,2);
    obj.x(:,1) = [0 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
    obj.x(:,2) = [0 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];

    disp('[obj_init]: Loaded Trajectory: Hover (default)');
end


end
