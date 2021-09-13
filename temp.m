A     = db.A;
B     = db.B;
I     = db.I;

q_c = [ q(1) ; -q(2) ; -q(3) ; -q(4) ];     % Conjugate quaternion

Jxx = (model.motor.m/12)*(3*(model.motor.r1^2+model.motor.r0^2)+model.motor.h^2);
Jyy = (model.motor.m/12)*(3*(model.motor.r1^2+model.motor.r0^2)+model.motor.h^2);
Jzz = (model.motor.m/2)*(model.motor.r1^2+model.motor.r0^2);
J = diag([Jxx Jyy Jzz]);                    % Motor Inertia Tensor


vb = quatrot2(v,q_c);                       % velocity in body frame

Qvb = sign(vb).*[vb.^0 abs(vb) vb.^2];      % quadratic structure of craft linear velocity (body frame)
Qw = sign(w).*[w.^0     abs(w)   w.^2];     % quadratic structure of craft angular velocity
Wm = [0       0      0      0;
      0       0      0      0;
    -wm(1) -wm(2) wm(3) wm(4)];             % Motor rotational speed map
% Torques
PwWm = [...
    cross(w,Wm(:,1))...             % Cross products of motor and craft angular velocities
    cross(w,Wm(:,2))...
    cross(w,Wm(:,3))...
    cross(w,Wm(:,4))];
tau_mot  =  F_w(2:4,1);                                                 % Motor
tau_rot  = -cross(w,I*w);                                               % Rotating Frame
tau_linD = -[ A(1,:)*Qvb(1,:)' ; A(2,:)*Qvb(2,:)'; A(3,:)*Qvb(3,:)'];   % Linear Velocity Drag
tau_accD = -[ B(1,:)*Qw(1,:)'  ; B(2,:)*Qw(2,:)' ; B(3,:)*Qw(3,:)'];    % Angular Velocity Drag
tau_g    = -J*(sum(PwWm,2));                                            % Motor Precession
tau_ext  = FT_ext(4:6,1);                                               % External
w_dot = I\(tau_mot + tau_rot + tau_g + tau_linD + tau_accD + tau_ext);
