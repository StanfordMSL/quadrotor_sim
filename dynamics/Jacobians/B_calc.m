function B = B_calc(in1,in2)
%B_CALC
%    B = B_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Sep-2021 03:20:02

s7 = in1(7,:);
s8 = in1(8,:);
s9 = in1(9,:);
s10 = in1(10,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
t2 = s7.*u2;
t3 = s7.*u3;
t4 = s8.*u2;
t5 = s7.*u4;
t6 = s8.*u3;
t7 = s9.*u2;
t8 = s8.*u4;
t9 = s9.*u3;
t10 = s10.*u2;
t11 = s9.*u4;
t12 = s10.*u3;
t13 = s10.*u4;
t14 = s7.^2;
t15 = s8.^2;
t16 = s9.^2;
t17 = s10.^2;
t18 = u2.^2;
t19 = u3.^2;
t20 = u4.^2;
t24 = s7.*4.0e+2;
t25 = s8.*4.0e+2;
t26 = s9.*4.0e+2;
t27 = s10.*4.0e+2;
t21 = -t7;
t22 = -t8;
t23 = -t12;
t28 = -t24;
t29 = t14+t15+t16+t17;
t31 = t18+t19+t20+1.6e+5;
t30 = t4+t9+t13+t28;
t32 = t2+t11+t23+t25;
t33 = t3+t10+t22+t26;
t34 = t5+t6+t21+t27;
t35 = 1.0./sqrt(t29);
t36 = 1.0./sqrt(t31);
t37 = t36.^3;
t38 = s7.*t35.*t36;
t39 = s8.*t35.*t36;
t40 = s9.*t35.*t36;
t41 = s10.*t35.*t36;
t42 = -t39;
t43 = -t40;
t44 = -t41;
B = reshape([0.0,0.0,0.0,s7.*s9.*2.794575471698113e-1+s8.*s10.*2.794575471698113e-1,s7.*s8.*(-2.794575471698113e-1)+s9.*s10.*2.794575471698113e-1,t14.*1.397287735849057e-1-t15.*1.397287735849057e-1-t16.*1.397287735849057e-1+t17.*1.397287735849057e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t42+t30.*t35.*t37.*u2,t38-t32.*t35.*t37.*u2,t41-t33.*t35.*t37.*u2,t43-t34.*t35.*t37.*u2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t43+t30.*t35.*t37.*u3,t44-t32.*t35.*t37.*u3,t38-t33.*t35.*t37.*u3,t39-t34.*t35.*t37.*u3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t44+t30.*t35.*t37.*u4,t40-t32.*t35.*t37.*u4,t42-t33.*t35.*t37.*u4,t38-t34.*t35.*t37.*u4,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[17,4]);
