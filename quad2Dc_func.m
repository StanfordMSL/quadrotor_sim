function x_dot = quad2Dc_func(in1,in2)
%QUAD2DC_FUNC
%    X_DOT = QUAD2DC_FUNC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    01-Oct-2021 00:52:56

u1 = in2(1,:);
u2 = in2(2,:);
x3 = in1(3,:);
x4 = in1(4,:);
x5 = in1(5,:);
x6 = in1(6,:);
x_dot = [x4;x5;x6;u1.*sin(x3).*(-1.0e+2./5.3e+1);u1.*cos(x3).*(1.0e+2./5.3e+1)-9.81e+2./1.0e+2;u2.*6.25e+2];
