function t_comp = jacLQR(x_upd,z_upd,input_mode)

tic

% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
switch input_mode
    case 'body_rate'
        % LQR Body Rate Control State and Inputs
        s = sym('s',[17 1],'real');
        x = sym('x',[10 1],'real');
        z = sym('z',[7 1],'real');
        u = sym('u',[4 1],'real');
        
        s_upd = [x_upd ; z_upd]; 
        s_upd = subs(s_upd,x,s(1:10));
        s_upd = subs(s_upd,z,s(11:17));  
end

% Jacobians
A = jacobian(s_upd,s);
B = jacobian(s_upd,u);

% Output Jacobian Functions
matlabFunction(A,'File','dynamics/Jacobians/A_calc','vars',{s,u});
matlabFunction(B,'File','dynamics/Jacobians/B_calc','vars',{s,u});    

t_comp = toc;

end