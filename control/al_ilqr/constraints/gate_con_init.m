function gate_con_init(gate,input_mode,model)

% Quadcopter Parameters (relative position of n_p points on body)
l_arm  = model.est.L;
r_d_arr = [ l_arm    0.00  -l_arm    0.00;
            0.00  -l_arm    0.00   l_arm; 
            0.00    0.00    0.00    0.00 ];
n_p = size(r_d_arr,2);

 % Initialize Variables
switch input_mode
    case 'direct'
        n_x = 13;
        n_u = 4;
    case 'wrench'
        n_x = 13;
        n_u = 4;
    case 'body_rate'
        n_x = 10;
        n_u = 4;
end
syms x [n_x 1] real
syms u [n_u 1] real

if (any(ismember(fields(gate),'p_ctr')) && (size(gate.p_ctr,2) > 0))
    % Gate Parameters
    p_G1 = gate.p_box(:,1);
    p_G2 = gate.p_box(:,2);
    p_G4 = gate.p_box(:,4);

    r_12 = p_G2 - p_G1;
    r_14 = p_G4 - p_G1;
    r_g_arr = [r_12 r_14];

    % Initialize Function Output
    syms gain [n_p*2 1] real % actual

    for j = 1:2
        r_g = r_g_arr(:,j);

        for k = 1:n_p
            idx = (j-1)*n_p+k;

            r_d = x(1:3,1) + quatrot2(r_d_arr(:,k),x(7:10,1));
            gain(idx,1) = dot((r_d-p_G1),r_g)./(r_g'*r_g);
        end
    end

    con = [-gain ; gain-ones(n_p*2,1)];
    con_x = jacobian(con,x);
    con_u = jacobian(con,u);
else
    % Initialize empty functions of x and u
    con   = x(1).*u(1).*zeros(n_p*4,1);
    con_x = x(1).*u(1).*zeros(n_p*4,n_x);
    con_u = x(1).*u(1).*zeros(n_p*4,n_u);
end

address = 'control/al_ilqr/constraints/';
matlabFunction(con,'File',[address,'gate_con'],'vars',{x,u})
matlabFunction(con_x,'File',[address,'gate_con_x'],'vars',{x,u})
matlabFunction(con_u,'File',[address,'gate_con_u'],'vars',{x,u})

end