function [t_comp,x_upd] = quad_EoM_br(model)

tic

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = sym('x',[10 1],'real');

% Inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u = sym('u',[4 1],'real');
 
% Useful Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

v = x(4:6,1);
q = x(7:10,1);

fn = u(1,1);
f  = fn2f(fn);

w = u(2:4,1);
W = [  0  -w(1) -w(2) -w(3) ;
     w(1)   0    w(3) -w(2) ;
     w(2) -w(3)    0   w(1) ;
     w(3)  w(2) -w(1)    0];                % Body Rate vee map


Qvw = sign(v).*[v.^0 abs(v) v.^2];          % quadratic structure of craft linear velocity (world frame)

% Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:2
    if k == 1       % Actual
        db = model.act;        
        quad_func = 'dynamics/flight/quadcopter_act';
    else            % Estimated
        db = model.est;     
        quad_func = 'dynamics/flight/quadcopter_est';
    end
    
    % Unpack
    g     = db.g;
    dt    = db.dt;
    D     = db.D;
    m     = db.m;
    
    % Forces
    F_g   = m.*[0 ; 0 ; -g];                                                % Gravity                                        
    F_t   = quatrot2([0 ; 0 ; f],q);                                        % Thrust
    F_D   = -[ D(1,:)*Qvw(1,:)' ; D(2,:)*Qvw(2,:)' ; D(3,:)*Qvw(3,:)'];     % Drag
    F_ext = sym('F_ext',[3 1],'real');                                      % External
    
    % Dynamics Equations 
    p_dot = v;
    v_dot = (1/m) .* ( F_g + F_t + F_D + F_ext);
    q_dot = 0.5*W*q;
    
    x_dot = [ p_dot ; v_dot ; q_dot];
    
    % Discrete Update Equations 
    x_upd = x + dt.*x_dot;
    x_upd(7:10,1) = x_upd(7:10,1)./norm(x_upd(7:10,1));    % ensure quaternion is still unit mag.
    
    % Output Quadcopter Dynamics Function
    x_upd = simplify(x_upd);
    
    matlabFunction(x_upd,'File',quad_func,'vars',{x,u,F_ext});
end
    
t_comp = toc;
end