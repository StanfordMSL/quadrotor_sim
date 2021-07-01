function map = map_init(map_id)

address = ['maps/',map_id,'.csv'];

% Gate Data
if isfile(address)
    data = readmatrix(address, 'Range', [2 2]);
    N_g = size(data,2);
    
    map.p_g  = zeros(3,N_g);    
    map.p_gc = zeros(3,4,N_g);
    map.q_star = zeros(4,N_g);
    for j = 1:N_g
        map.p_g(:,j)  = data(1:3,j);
        
        dim_gate = zeros(3,4);
        for k = 1:4
            idx = 3*(k-1) + 8;
            dim_gate(:,k) = data(idx:idx+2,j);
        end
        % Generate conjugate because because matlab uses the frame rotation convention
        q_star = quatconj(data(4:7,j)');    
        map.q_star(:,j) = q_star';
        map.p_gc(:,:,j) = data(1:3,j)+quatrotate(q_star,dim_gate')';
    end
    
    disp(['[map_init]: Loaded Map: ',map_id]);
else
    disp('[map_init]: Loaded Map: Empty Flightroom (default)');
end

% Map Limits
map.x_lim = [-8.1 8.1];
map.y_lim = [-3.2 3.2];
map.z_lim = [0 3];


end
