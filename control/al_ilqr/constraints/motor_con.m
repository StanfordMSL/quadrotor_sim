function con = motor_con(in1,in2,in3)
%MOTOR_CON
%    CON = MOTOR_CON(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    06-Jul-2021 15:08:58

u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
up2 = in3(2,:);
up3 = in3(3,:);
up4 = in3(4,:);
t2 = u4.*2.3e+1;
t3 = up4.*2.3e+1;
t4 = u1./4.0;
t8 = u2.*(4.85e+2./5.9e+1);
t9 = u3.*(4.85e+2./5.9e+1);
t10 = up2.*(4.85e+2./5.9e+1);
t11 = up3.*(4.85e+2./5.9e+1);
t16 = u2.*u4.*7.372881355932203e-2;
t17 = u3.*u4.*7.372881355932203e-2;
t5 = -t2;
t6 = -t3;
t7 = -t4;
t12 = -t8;
t13 = -t9;
t14 = -t10;
t15 = -t11;
t18 = -t16;
t19 = -t17;
con = [t3+t4+t5+t10+t11+t12+t13+t16+t19-5.160224;t3+t4+t5+t8+t9+t14+t15+t17+t18-5.160224;t2+t4+t6+t8+t11+t13+t14+t16+t17-5.160224;t2+t4+t6+t9+t10+t12+t15+t18+t19-5.160224;t2+t6+t7+t8+t9+t14+t15+t17+t18;t2+t6+t7+t10+t11+t12+t13+t16+t19;t3+t5+t7+t9+t10+t12+t15+t18+t19;t3+t5+t7+t8+t11+t13+t14+t16+t17];
