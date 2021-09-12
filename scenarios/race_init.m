function obj = race_init(mission,misc)

obj.type = 'race';

gt_add = 'scenarios/gates/';
ms_add = ['scenarios/missions/',mission,'.csv'];

% Map
obj.map.act = [
    -8.1 8.1;       % Actual Map x-limits (length)
    -3.2 3.2;       % Actual  Map y-limits (width)
     0.0 3.0];      % Actual  Map z-limits (height)      
obj.map.lim  = [
    -7.5 7.5;       % Traj Map x-limits (length)
    -2.7 2.7;       % Traj Map y-limits (width)
     0.5 2.5];      % Traj Map z-limits (height)
 
% Gates
gates = dir([gt_add '*.csv']);
for k_wp = 1:length(gates)
  file = [gt_add gates(k_wp).name];
  obj.db(k_wp).gt_dim = readmatrix(file,'Range', [1 2]);
end

% Missions
if isfile(ms_add)
    % Raw mission data
    data = readmatrix(ms_add, 'Range', [1 2]);
    
    % Some useful intermediate terms
    N = size(data,2);
    N_kf = sum(data(1,:)==0);
    N_gt = N-N_kf;
    
    % Mission flatoutputs, keyframes and gates
    obj.kf.t   = zeros(1,N);
    obj.kf.fo  = zeros(4,2,N);
    obj.kf.x   = zeros(13,N_kf);
    obj.kf.gt  = zeros(8,N_gt);
    
    q = [-1 ; 0 ; 0 ; 0];
    k_kf = 1;
    k_gt = 1;
    for k_wp = 1:N
        if (data(1,k_wp) == 0)
            % Keyframes
            obj.kf.x(:,k_kf) = data(2:14,k_wp);
            k_kf = k_kf + 1;
            q = data(8:11,k_wp) ;
        else
            % Gates
            obj.kf.gt(:,k_gt) = [data(1,k_wp) ; data(2:4,k_wp) ; data(8:11,k_wp) ];
            k_gt = k_gt + 1;
        end
        
        % Flat Outputs
        eul = quat2eul(q');
        obj.kf.fo(1:3,1,k_wp) = data(2:4,k_wp);
        obj.kf.fo(1:3,2,k_wp) = data(5:7,k_wp);
        obj.kf.fo(4,1,k_wp)   = eul(1);
        obj.kf.fo(4,2,k_wp)   = data(14,k_wp);
        
        if k_wp > 1
            s_int = norm(obj.kf.fo(1:3,1,k_wp) - obj.kf.fo(1:3,1,k_wp-1));
            if s_int == 0
                t_int = misc.t_hov;      % to catch the hover case
            else
                t_int = round(s_int/misc.v_cr,1);
            end
            obj.kf.t(1,k_wp) = obj.kf.t(1,k_wp-1) + t_int;
        end
    end
    
    disp(['[race_init]: Loaded Mission: ', mission]);
else
    obj.kf.x = zeros(13,2);
    obj.kf.x(7,:) = 1;
    
    disp('[race_init]: Mission input not recognized. Defaulting to hover at origin');
end

end
