function [con,con_x,con_u] = room_con(x,map)

% Unpack some stuff
pos_min = [map.x_lim(1,1) ; map.y_lim(1,1) ; map.z_lim(1,1) ];
pos_max = [map.x_lim(1,2) ; map.y_lim(1,2) ; map.z_lim(1,2) ];
pos = x(1:3,:);

% Count
N = size(x,2);
n_c = size(pos,1);
n_x = size(x,1);
n_u = 4;

% Initialize Output
con   = zeros(2*n_c,N);
con_x = zeros(2*n_c,n_x,N);
con_u = zeros(2*n_c,n_u,N);

% Compute Constraints and Their Partials
for k = 1:N
    con_min = -pos(:,k) + pos_min + 0.1;
    con_max =  pos(:,k) - pos_max - 0.1;
    con(:,k) = [con_min ; con_max];

    con_x_min = [-eye(3) zeros(3,10)];
    con_x_max = [ eye(3) zeros(3,10)];
    con_x(:,:,k) = [con_x_min ; con_x_max];
end
    
end