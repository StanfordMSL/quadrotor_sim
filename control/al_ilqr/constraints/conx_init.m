function conx_init(model,input_mode)

tic

% Quadcopter Parameters (relative position of n_p points on body)
l_arm  = model.est.L;
r_d_arr = [ l_arm    0.00  -l_arm    0.00;
            0.00  -l_arm    0.00   l_arm; 
            0.00    0.00    0.00    0.00 ];
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
p_box = sym('p_box',[3 4],'real');
gain = sym('gain',[n_p*2 1],'real');

p_G1 = p_box(:,1);
p_G2 = p_box(:,2);
p_G4 = p_box(:,4);
r_g_arr = [p_G2-p_G1 p_G4-p_G1];

for j = 1:2
    r_g = r_g_arr(:,j);

    for k = 1:n_p
        idx = (j-1)*n_p+k;

        r_d = x(1:3,1) + quatrot2(r_d_arr(:,k),x(7:10,1));
        gain(idx,1) = dot((r_d-p_G1),r_g)./(r_g'*r_g);
    end
end

conx_gate   = [-gain ; gain-ones(n_p*2,1)];

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
conx_map_x = jacobian(conx_map,x);

address = 'control/al_ilqr/constraints/';
matlabFunction(conx_gate,'File',[address,'conx_gate'],'vars',{x,p_box});
matlabFunction(conx_gate_x,'File',[address,'conx_gate_x'],'vars',{x,p_box});

matlabFunction(conx_map,'File',[address,'conx_map'],'vars',{x,map_lim});
matlabFunction(conx_map_x,'File',[address,'conx_map_x'],'vars',{x,map_lim});

t_comp = toc;

disp(['[conx_init]: State Constraints Generated in ' num2str(t_comp) 's'])

end