function quad_gen(model,type)
%#codegen
% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = sym('p',[3 1],'real');
v = sym('v',[3 1],'real');
q = sym('q',[4 1],'real');
w = sym('w',[3 1],'real');

% Input (wrench) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wr = sym('wr',[4 1],'real');
    
% Model Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt  = sym('dt','real');
m   = sym('m','real'); 
Ipp = sym('Ipp',[3 1],'real');
Ipd = sym('Ipd',[3 1],'real');
Df  = sym('Df',[3 3],'real');
g   = sym('g','real'); 

% Useful Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

I = [Ipp(1) Ipd(1) Ipd(2);
     Ipd(1) Ipp(2) Ipd(3);
     Ipd(2) Ipd(3) Ipp(3)]; 

W = [  0  -w(1) -w(2) -w(3) ;
     w(1)   0    w(3) -w(2) ;
     w(2) -w(3)    0   w(1) ;
     w(3)  w(2) -w(1)    0];                % Body Rate vee map
x = [ p ; v ; q ; w ];                      % EoM State Vector

Qvw = sign(v).*[v.^2 abs(v) v.^0];          % quadratic structure of craft linear velocity (world frame)

% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Forces
F_g   = m.*[0 ; 0 ; -g];                                              % Gravity                                        
F_t   = quatrot2([0 ; 0 ; wr(1,1)],q);                                 % Thrust
F_D   = -[ Df(1,:)*Qvw(1,:)' ; Df(2,:)*Qvw(2,:)' ; Df(3,:)*Qvw(3,:)'];     % Drag

% Torques
tau_mot  =  wr(2:4,1);                                                 % Motor
tau_rot  = -cross(w,I*w);                                             % Rotating Frame

% Dynamics Equations 
p_dot = v;
v_dot = (1/m) .* (F_g + F_t + F_D);
q_dot = 0.5*W*q;
w_dot = I\(tau_mot + tau_rot);

x_dot = [ p_dot ; v_dot ; q_dot ; w_dot ];

% Discrete Update Equations 
x_upd = x + dt.*x_dot;    
x_upd(7:10,1)  =  x_upd(7:10,1)./norm(x_upd(7:10,1));                   % ensure quaternion is still unit mag.

% Substistute Parameters Based on Type %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% We always assume these parameters to be constant
x_upd = subs(x_upd,Ipd,model.act.Ipd);
x_upd = subs(x_upd,g,model.act.g);
x_upd = subs(x_upd,Df(1:3,2:3),model.act.Df(1:3,2:3));

switch type
    case 'simulation'
        func = 'flight/matlab/codegen/quad_sim';
        
        theta = [m ; Ipp ; Df(1:3,1)];
        u = wr;

        x_upd = simplify(x_upd);

        matlabFunction(x_upd,'File',func,'vars',{x,u,theta,dt});
end


