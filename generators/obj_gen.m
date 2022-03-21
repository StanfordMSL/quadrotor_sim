function obj = obj_gen(keyframes)

kf_add = ['data/scenarios/keyframes/',keyframes,'.csv'];

% Keyframes
if isfile(kf_add)    
    obj.x = readmatrix(kf_add, 'Range', [2 2]);
    
    disp(['[obj_gen]: Loaded Keyframe Sequence: ',keyframes]);
else
    kf_add = 'scenarios/keyframes/hover.csv';
    obj.x = readmatrix(kf_add, 'Range', [2 2]);

    obj.x = zeros(13,2);
    obj.x(7,:) = 1;
    
    disp('[obj_gen]: Keyframe input not recognized. Defaulting to hover at origin');
end

% Map
obj.map = [
    -8.1 8.1;       % Map x-limits (length)
    -3.2 3.2;       % Map y-limits (width)
     0 3];          % Map z-limits (height)      
