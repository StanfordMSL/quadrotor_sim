function gate_con_init(map,input_mode,model)

switch input_mode
    case 'direct'
        syms x [13 1] real
    case 'body_rate'
        syms x [10 1] real
end
syms u [n_u 1] real

% Initialize Function Output
syms gain [8 1] real

% Gate Parameters
p_G1 = map.p_gc(:,1);
p_G2 = map.p_gc(:,2);
p_G4 = map.p_gc(:,4);

r_12 = p_G2 - p_G1;
r_14 = p_G4 - p_G1;
r_g_arr = [r_12 r_14];

% Quadcopter Dimensions
l_arm  = model.est.L;

% Relative position of the four points to body center in body frame
r_d_arr = [ l_arm    0.00  -l_arm    0.00;
            0.00  -l_arm    0.00   l_arm; 
            0.00    0.00    0.00    0.00 ];

for j = 1:2
    r_g = r_g_arr(:,j);
 
    for k = 1:4
        idx = (j-1)*4+k;
        
        r_d = quatrot2(r_d_arr(:,k),x(7:10));

        gain(idx,1) = dot((r_d-p_G1),r_g)./(r_g'*r_g);
    end
end

con = [-gain ; gain-ones(8,1)];
con_x = jacobian(con,x);
con_u = jacobian(con,u);

address = 'controllers/high_level/al_ilqr/constraints/';
matlabFunction(con,'File',[address,'gate_con'],'vars',{x,u})
matlabFunction(con_x,'File',[address,'gate_con_x'],'vars',{x,u})
matlabFunction(con_u,'File',[address,'gate_con_u'],'vars',{x,u})

end