function obj = race_init(keyframes,gates)

obj.type = 'race';

kf_add = ['scenarios/keyframes/',keyframes,'.csv'];
gt_add = ['scenarios/gates/',gates,'.csv'];

% Keyframes
if isfile(kf_add)    
    obj.kf.x = readmatrix(kf_add, 'Range', [2 2]);
    
    disp(['[race_init]: Loaded Keyframe Sequence: ',keyframes]);
else
    obj.kf.x = zeros(13,2);
    obj.kf.x(7,:) = 1;
    
    disp('[race_init]: Keyframe input not recognized. Defaulting to hover at origin');
end

if isfile(gt_add)
    data = readmatrix(gt_add, 'Range', [2 2]);
    N_g = size(data,2);
    
    obj.gt.p_ctr = zeros(3,N_g);    
    obj.gt.p_box = zeros(3,4,N_g);
    obj.gt.q_box = zeros(4,N_g);
    obj.gt.seq  = zeros(1,N_g);
    
    if N_g == 0
        % Empty room.
    else
        for j = 1:N_g
            obj.gt.p_ctr(:,j)  = data(1:3,j);
            
            dim_gate = zeros(3,4);
            for k = 1:4
                idx = 3*(k-1) + 8;
                dim_gate(:,k) = data(idx:idx+2,j);
            end
            % Generate conjugate because because matlab uses the frame rotation convention
            q_star = quatconj(data(4:7,j)');
            obj.gt.q_box(:,j)   = q_star';
            obj.gt.p_box(:,:,j) = data(1:3,j)+quatrotate(q_star,dim_gate')';
            
            obj.gt.seq(1,j) = data(20,j);
        end
    end
    disp(['[race_init]: Loaded Gate Sequence: ',gates]);  
else
    obj.gt.p_ctr = zeros(3,0);    
    obj.gt.p_box = zeros(3,4,0);
    obj.gt.q_box = zeros(4,0);
    obj.gt.seq  = zeros(1,0);
    
    disp('[race_init]: Gate input not recognized. Defaulting to empty room');
end
