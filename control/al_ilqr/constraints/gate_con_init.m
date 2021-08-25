function gate_con_init(model,input_mode)

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

conx   = [-gain ; gain-ones(n_p*2,1)];
conx_x = jacobian(conx,x);

address = 'control/al_ilqr/constraints/';
matlabFunction(conx,'File',[address,'conx'],'vars',{x,p_box});
matlabFunction(conx_x,'File',[address,'conx_x'],'vars',{x,p_box});

t_comp = toc;

disp(['[gate_con_init]: Gate Constraints Generated in ' num2str(t_comp) 's'])

end