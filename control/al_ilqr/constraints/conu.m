function conu = conu(in1,in2)
%CONU
%    CONU = CONU(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    26-Aug-2021 11:08:19

u1 = in1(1,:);
u2 = in1(2,:);
u3 = in1(3,:);
u4 = in1(4,:);
up2 = in2(2,:);
up3 = in2(3,:);
up4 = in2(4,:);
t2 = u4.*4.6e+1;
t3 = up4.*4.6e+1;
t6 = u2.*(9.7e+2./5.9e+1);
t7 = u3.*(9.7e+2./5.9e+1);
t8 = up2.*(9.7e+2./5.9e+1);
t9 = up3.*(9.7e+2./5.9e+1);
t14 = u2.*u4.*7.372881355932203e-2;
t15 = u3.*u4.*7.372881355932203e-2;
t18 = u1.*5.28865;
t4 = -t2;
t5 = -t3;
t10 = -t6;
t11 = -t7;
t12 = -t8;
t13 = -t9;
t16 = -t14;
t17 = -t15;
t19 = -t18;
mt1 = [t3+t4+t8+t9+t10+t11+t14+t17+t18-5.28865,t3+t4+t6+t7+t12+t13+t15+t16+t18-5.28865,t2+t5+t6+t9+t11+t12+t14+t15+t18-5.28865,t2+t5+t7+t8+t10+t13+t16+t17+t18-5.28865,t2+t5+t6+t7+t12+t13+t15+t16+t19,t2+t5+t8+t9+t10+t11+t14+t17+t19];
mt2 = [t3+t4+t7+t8+t10+t13+t16+t17+t19,t3+t4+t6+t9+t11+t12+t14+t15+t19];
conu = reshape([mt1,mt2],8,1);
