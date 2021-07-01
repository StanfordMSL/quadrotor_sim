function gate_con_init(map,input_mode,model)

% Tuning Parameter
tol = 0.2;

% Gate Parameters
p_g  = map.p_g;
p_G1 = map.p_gc(:,1);
p_G2 = map.p_gc(:,2);
p_G4 = map.p_gc(:,4);

r_12 = p_G2 - p_G1;
r_14 = p_G4 - p_G1;
r_g_arr = [r_12 r_14];

% Quadcopter Parameters (relative position of n_p points on body)
l_arm  = model.est.L;
r_d_arr = [ l_arm    0.00  -l_arm    0.00;
            0.00  -l_arm    0.00   l_arm; 
            0.00    0.00    0.00    0.00 ];
n_p = size(r_d_arr,2);

% Initialize Variables
switch input_mode
    case 'direct'
        syms x [13 1] real
    case 'wrench'
        syms x [13 1] real
    case 'body_rate'
        syms x [10 1] real
end
syms u [4 1] real

% Initialize Function Output
syms gain [n_p*2 1] real % actual

for j = 1:2
    r_g = r_g_arr(:,j);
 
    for k = 1:n_p
        idx = (j-1)*n_p+k;
        
        r_d = x(1:3,1) + quatrot2(r_d_arr(:,k),x(7:10,1));

%         a1 = 1/tol;
%         a2 = p_g(1,1)/tol;
%         a3 = 1;
%         
%         g = -(a1*x(1,1)+a2)^2 + a3;
        g = 1;
        
        gain(idx,1) = g.*dot((r_d-p_G1),r_g)./(r_g'*r_g);
    end
end

con = [-gain ; gain-ones(n_p*2,1)];
con_x = jacobian(con,x);
con_u = jacobian(con,u);

address = 'controllers/high_level/al_ilqr/constraints/';
matlabFunction(con,'File',[address,'gate_con'],'vars',{x,u})
matlabFunction(con_x,'File',[address,'gate_con_x'],'vars',{x,u})
matlabFunction(con_u,'File',[address,'gate_con_u'],'vars',{x,u})

end