function A = A_calc(in1,in2)
%A_CALC
%    A = A_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Sep-2021 03:20:02

s7 = in1(7,:);
s8 = in1(8,:);
s9 = in1(9,:);
s10 = in1(10,:);
u1 = in2(1,:);
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
t30 = s7.*u1.*2.794575471698113e-1;
t31 = s8.*u1.*2.794575471698113e-1;
t32 = s9.*u1.*2.794575471698113e-1;
t33 = s10.*u1.*2.794575471698113e-1;
t21 = -t7;
t22 = -t8;
t23 = -t12;
t28 = -t24;
t29 = t14+t15+t16+t17;
t34 = -t31;
t36 = t18+t19+t20+1.6e+5;
t35 = t4+t9+t13+t28;
t37 = t2+t11+t23+t25;
t38 = t3+t10+t22+t26;
t39 = t5+t6+t21+t27;
t40 = 1.0./sqrt(t29);
t42 = 1.0./sqrt(t36);
t41 = t40.^3;
t43 = t40.*t42.*u2;
t44 = t40.*t42.*u3;
t45 = t40.*t42.*u4;
t49 = t40.*t42.*4.0e+2;
t46 = -t43;
t47 = -t44;
t48 = -t45;
A = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,9.99999999990566e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,9.99999999990566e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,9.99999999990566e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t32,t34,t30,t49+s7.*t35.*t41.*t42,t43-s7.*t37.*t41.*t42,t44-s7.*t38.*t41.*t42,t45-s7.*t39.*t41.*t42,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,0.0,0.0,0.0,0.0,t33,-t30,t34,t46+s8.*t35.*t41.*t42,t49-s8.*t37.*t41.*t42,t48-s8.*t38.*t41.*t42,t44-s8.*t39.*t41.*t42,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,0.0,0.0,0.0,t30,t33,-t32,t47+s9.*t35.*t41.*t42,t45-s9.*t37.*t41.*t42,t49-s9.*t38.*t41.*t42,t46-s9.*t39.*t41.*t42,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,0.0,0.0,t31,t32,t33,t48+s10.*t35.*t41.*t42,t47-s10.*t37.*t41.*t42,t43-s10.*t38.*t41.*t42,t49-s10.*t39.*t41.*t42,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0],[17,17]);
