function con = motor_con(in1,in2,in3)
%MOTOR_CON
%    CON = MOTOR_CON(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    14-Jul-2021 15:43:35

u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
up2 = in3(2,:);
up3 = in3(3,:);
up4 = in3(4,:);
t2 = u1./4.0;
t4 = u2.*(7.7e+1./1.77e+2);
t5 = u3.*(7.7e+1./1.77e+2);
t6 = u4.*(2.51e+2./4.0e+2);
t7 = up2.*(7.7e+1./1.77e+2);
t8 = up3.*(7.7e+1./1.77e+2);
t9 = up4.*(2.51e+2./4.0e+2);
t16 = u2.*u4.*2.740112994350282e-3;
t17 = u3.*u4.*2.740112994350282e-3;
t3 = -t2;
t10 = -t4;
t11 = -t5;
t12 = -t6;
t13 = -t7;
t14 = -t8;
t15 = -t9;
t18 = -t16;
t19 = -t17;
con = [t2+t7+t8+t9+t10+t11+t12+t16+t19-6.602205000000001;t2+t4+t5+t9+t12+t13+t14+t17+t18-6.602205000000001;t2+t4+t6+t8+t11+t13+t15+t16+t17-6.602205000000001;t2+t5+t6+t7+t10+t14+t15+t18+t19-6.602205000000001;t3+t4+t5+t6+t13+t14+t15+t17+t18+3.6552e-5;t3+t6+t7+t8+t10+t11+t15+t16+t19+3.6552e-5;t3+t5+t7+t9+t10+t12+t14+t18+t19+3.6552e-5;t3+t4+t8+t9+t11+t12+t13+t16+t17+3.6552e-5];
