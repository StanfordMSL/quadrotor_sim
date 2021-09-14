function conx_init(model,input_mode)

tic

% Quadcopter Parameters (relative position of n_p points on body)
l_x = model.est.dim(1);
l_y = model.est.dim(2);
l_z = model.est.dim(3);

% r_d_arr = [-l_x-0.1   l_x+0.15   l_x+0.15 -l_x-0.1 -l_x-0.1  l_x+0.15   l_x+0.15  -l_x-0.1;
%               0.03      0.03      -0.03      -0.03    0.03      0.03      -0.03      -0.03;
%               l_z       l_z       l_z       l_z      -l_z      -l_z      -l_z      -l_z];
r_d_arr = [-l_x   l_x   l_x  -l_x  -l_x   l_x   l_x  -l_x;
            l_y   l_y  -l_y  -l_y   l_y   l_y  -l_y  -l_y;
            l_z   l_z   l_z   l_z  -l_z  -l_z  -l_z  -l_z];
n_p = size(r_d_arr,2);

% Variable dimensions based on Input
switch input_mode
    case 'direct'
        n_x = 13;
    case 'wrench'
        n_x = 13;
    case 'body_rate'
        n_x = 17;
end

% Initialize Input/Outputs
x = sym('x',[n_x 1],'real');

% Gate Constraint
p0 = sym('p0',[3 1],'real');
p1 = sym('p1',[3 1],'real');
p2 = sym('p2',[3 1],'real');

p12 = p2-p1;
p10 = p0-p1;

t  = (p10'*p12)/(p12'*p12);
pn = p1 + t*(p2-p1);
n  = p0-pn;

conx_gate = sym('conx_gate',[n_p 1],'real');
for k = 1:n_p    
    r_d = x(1:3,1) + quatrot2(r_d_arr(:,k),x(7:10,1));
    
    conx_gate(k,1) = -(n(1)*(r_d(1)-pn(1))+n(2)*(r_d(2)-pn(2))+ n(3)*(r_d(3)-pn(3)));
end

% Plane Constraint
c = sym('c',[4 1],'real');
x_d = x(1:3,1);

num = abs(c(1:3)'*x_d+c(4));
den = norm(c(1:3));
conx_plane = num/den;

% Map Constraint
map_lim = sym('map_lim',[3 2],'real');

conx_map = [
    -x(1)+map_lim(1,1) ;
     x(1)-map_lim(1,2) ;
    -x(2)+map_lim(2,1) ;
     x(2)-map_lim(2,2) ;
    -x(3)+map_lim(3,1) ;
     x(3)-map_lim(3,2) ]; 

% Combine them
conx_gate_x = jacobian(conx_gate,x);
conx_plane_x = jacobian(conx_plane,x);
conx_map_x = jacobian(conx_map,x);

address = 'control/al_ilqr/constraints/';
matlabFunction(conx_gate,'File',[address,'conx_gate'],'vars',{x,p0,p1,p2});
matlabFunction(conx_gate_x,'File',[address,'conx_gate_x'],'vars',{x,p0,p1,p2});

matlabFunction(conx_plane,'File',[address,'conx_plane'],'vars',{x,c});
matlabFunction(conx_plane_x,'File',[address,'conx_plane_x'],'vars',{x,c});

matlabFunction(conx_map,'File',[address,'conx_map'],'vars',{x,map_lim});
matlabFunction(conx_map_x,'File',[address,'conx_map_x'],'vars',{x,map_lim});

t_comp = toc;

disp(['[conx_init]: State Constraints Generated in ' num2str(t_comp) 's'])

end