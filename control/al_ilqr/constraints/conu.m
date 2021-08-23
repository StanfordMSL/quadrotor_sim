function conu = conu(in1,in2)
%CONU
%    CONU = CONU(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    22-Aug-2021 19:30:09

u1 = in1(1,:);
u2 = in1(2,:);
u3 = in1(3,:);
u4 = in1(4,:);
up2 = in2(2,:);
up3 = in2(3,:);
up4 = in2(4,:);
t2 = u3.*(4.1e+1./3.0e+1);
t3 = u2.*(3.7e+1./4.0e+1);
t4 = up3.*(4.1e+1./3.0e+1);
t5 = up2.*(3.7e+1./4.0e+1);
t10 = u3.*u4.*(3.0./8.0e+2);
t12 = u2.*u3.*8.439490445859873e-3;
t13 = u4.*8.089171974522293;
t14 = up4.*8.089171974522293;
t18 = u2.*u4.*5.958333333333333e-3;
t19 = u1.*4.2446875;
t6 = -t2;
t7 = -t3;
t8 = -t4;
t9 = -t5;
t11 = -t10;
t15 = -t12;
t16 = -t13;
t17 = -t14;
t20 = -t18;
t21 = -t19;
conu = [t4+t5+t6+t7+t11+t14+t15+t16+t18+t19-4.2446875;t2+t3+t8+t9+t10+t14+t15+t16+t19+t20-4.2446875;t3+t4+t6+t9+t10+t12+t13+t17+t18+t19-4.2446875;t2+t5+t7+t8+t11+t12+t13+t17+t19+t20-4.2446875;t2+t3+t8+t9+t10+t12+t13+t17+t20+t21;t4+t5+t6+t7+t11+t12+t13+t17+t18+t21;t2+t5+t7+t8+t11+t14+t15+t16+t20+t21;t3+t4+t6+t9+t10+t14+t15+t16+t18+t21];
