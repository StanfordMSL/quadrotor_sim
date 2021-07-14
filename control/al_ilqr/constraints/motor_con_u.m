function con_u = motor_con_u(in1,in2,in3)
%MOTOR_CON_U
%    CON_U = MOTOR_CON_U(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Jul-2021 16:42:25

u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
t2 = u2.*2.740112994350282e-3;
t3 = u3.*2.740112994350282e-3;
t4 = u4.*2.740112994350282e-3;
t5 = -t2;
t6 = -t3;
t7 = -t4;
t8 = t4+7.7e+1./1.77e+2;
t10 = t4-7.7e+1./1.77e+2;
t9 = t7+7.7e+1./1.77e+2;
t11 = t7-7.7e+1./1.77e+2;
con_u = reshape([1.0./4.0,1.0./4.0,1.0./4.0,1.0./4.0,-1.0./4.0,-1.0./4.0,-1.0./4.0,-1.0./4.0,t10,t9,t8,t11,t9,t10,t11,t8,t11,t8,t10,t9,t8,t11,t9,t10,t2+t6-2.51e+2./4.0e+2,t3+t5-2.51e+2./4.0e+2,t2+t3+2.51e+2./4.0e+2,t5+t6+2.51e+2./4.0e+2,t3+t5+2.51e+2./4.0e+2,t2+t6+2.51e+2./4.0e+2,t5+t6-2.51e+2./4.0e+2,t2+t3-2.51e+2./4.0e+2],[8,4]);
