function [t_comp,x_upd] = quad_EoM(model)

tic

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = sym('p',[3 1],'real');
v = sym('v',[3 1],'real');
q = sym('q',[4 1],'real');
w = sym('w',[3 1],'real');

% Motor Inputs (rad/s) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wm = sym('wm',[4 1],'real');

% Noise %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wt = sym('wt',[13 1],'real');

% External Forces/Torques %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FT_ext = sym('FT_ext',[6 1],'real');
    
% Useful Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

W = [  0  -w(1) -w(2) -w(3) ;
     w(1)   0    w(3) -w(2) ;
     w(2) -w(3)    0   w(1) ;
     w(3)  w(2) -w(1)    0];                % Body Rate vee map

x = [ p ; v ; q ; w ];                      % EoM State Vector

q_c = [ q(1) ; -q(2) ; -q(3) ; -q(4) ];     % Conjugate quaternion

Jxx = (model.motor.m/12)*(3*(model.motor.r1^2+model.motor.r0^2)+model.motor.h^2);
Jyy = (model.motor.m/12)*(3*(model.motor.r1^2+model.motor.r0^2)+model.motor.h^2);
Jzz = (model.motor.m/2)*(model.motor.r1^2+model.motor.r0^2);
J = diag([Jxx Jyy Jzz]);                    % Motor Inertia Tensor

Qvw = sign(v).*[v.^0 abs(v) v.^2];          % quadratic structure of craft linear velocity (world frame)
vb = quatrot2(v,q_c);                       % velocity in body frame
Qvb = sign(vb).*[vb.^0 abs(vb) vb.^2];      % quadratic structure of craft linear velocity (body frame)
Qw = sign(w).*[w.^0     abs(w)   w.^2];     % quadratic structure of craft angular velocity
Wm = [0       0      0      0;
      0       0      0      0;
    -wm(1) -wm(2) wm(3) wm(4)];             % Motor rotational speed map

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
    A     = db.A;
    B     = db.B;
    m     = db.m;
    I     = db.I;
    m2w   = db.m2w;
    
    % Intermediate Terms (that use actual/estimated specific variables)
    F_m = m2f(wm);                      % Motor thrust values
    F_w = m2w*F_m;                      % Wrench mapped from motor thrust values
    PwWm = [...
        cross(w,Wm(:,1))...             % Cross products of motor and craft angular velocities
        cross(w,Wm(:,2))...
        cross(w,Wm(:,3))...
        cross(w,Wm(:,4))];    
    
    % Forces
    F_g   = m.*[0 ; 0 ; -g];                                                % Gravity                                        
    F_t   = quatrot2([0 ; 0 ; F_w(1,1)],q);                                 % Thrust
    F_D   = -[ D(1,:)*Qvw(1,:)' ; D(2,:)*Qvw(2,:)' ; D(3,:)*Qvw(3,:)'];     % Drag
    F_ext = FT_ext(1:3,1);                                                  % External
    
    % Torques
    tau_mot  =  F_w(2:4,1);                                                 % Motor
    tau_rot  = -cross(w,I*w);                                               % Rotating Frame
    tau_linD = -[ A(1,:)*Qvb(1,:)' ; A(2,:)*Qvb(2,:)'; A(3,:)*Qvb(3,:)'];   % Linear Velocity Drag
    tau_accD = -[ B(1,:)*Qw(1,:)'  ; B(2,:)*Qw(2,:)' ; B(3,:)*Qw(3,:)'];    % Angular Velocity Drag   
    tau_g    = -J*(sum(PwWm,2));                                            % Motor Precession
    tau_ext  = FT_ext(4:6,1);                                               % External 

    % Dynamics Equations 
    p_dot = v;
    v_dot = (1/m) .* ( F_g + F_t + F_D + F_ext);
    q_dot = 0.5*W*q;
    w_dot = I\(tau_mot + tau_rot + tau_g + tau_linD + tau_accD + tau_ext);
    
    x_dot = [ p_dot ; v_dot ; q_dot ; w_dot ];
    
    % Discrete Update Equations 
    x_upd = x + dt.*x_dot;
    
    x_upd(7:10,1)  =  x_upd(7:10,1)./norm(x_upd(7:10,1));        % ensure quaternion is still unit mag.
    x_upd = x_upd + wt;                                     % expose noise variable (model noise)
    
    % Output Quadcopter Dynamics Function
    x_upd = simplify(x_upd);
    
    matlabFunction(x_upd,'File',quad_func,'vars',{x,wm,FT_ext,wt});
end
    
t_comp = toc;
end