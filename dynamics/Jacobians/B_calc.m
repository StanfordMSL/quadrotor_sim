function B = B_calc(in1,in2)
%B_CALC
%    B = B_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    14-Jul-2021 15:43:29

u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
t2 = -x7;
t3 = (u2.*x7)./2.0e+2;
t4 = (u2.*x8)./2.0e+2;
t5 = (u3.*x7)./2.0e+2;
t6 = (u2.*x9)./2.0e+2;
t7 = (u3.*x8)./2.0e+2;
t8 = (u4.*x7)./2.0e+2;
t9 = (u2.*x10)./2.0e+2;
t10 = (u3.*x9)./2.0e+2;
t11 = (u4.*x8)./2.0e+2;
t12 = (u3.*x10)./2.0e+2;
t13 = (u4.*x9)./2.0e+2;
t14 = (u4.*x10)./2.0e+2;
t15 = -t6;
t16 = -t11;
t17 = -t12;
t27 = t2+t4+t10+t14;
t18 = t3+t13+t17+x8;
t19 = t5+t9+t16+x9;
t20 = t7+t8+t15+x10;
t28 = abs(t27);
t32 = sign(t27);
t21 = abs(t18);
t22 = abs(t19);
t23 = abs(t20);
t24 = sign(t18);
t25 = sign(t19);
t26 = sign(t20);
t33 = t28.^2;
t46 = (t28.*t32.*x8)./1.0e+2;
t47 = (t28.*t32.*x9)./1.0e+2;
t48 = (t28.*t32.*x10)./1.0e+2;
t29 = t21.^2;
t30 = t22.^2;
t31 = t23.^2;
t34 = (t21.*t24.*x7)./1.0e+2;
t35 = (t22.*t25.*x7)./1.0e+2;
t36 = (t23.*t26.*x7)./1.0e+2;
t37 = (t22.*t25.*x8)./1.0e+2;
t38 = (t23.*t26.*x8)./1.0e+2;
t39 = (t21.*t24.*x9)./1.0e+2;
t40 = (t23.*t26.*x9)./1.0e+2;
t41 = (t21.*t24.*x10)./1.0e+2;
t42 = (t22.*t25.*x10)./1.0e+2;
t43 = -t37;
t44 = -t40;
t45 = -t41;
t49 = t29+t30+t31+t33;
t50 = 1.0./sqrt(t49);
t59 = t34+t42+t44+t46;
t60 = t35+t38+t45+t47;
t61 = t36+t39+t43+t48;
t51 = t50.^3;
t52 = (t50.*x7)./2.0e+2;
t53 = (t50.*x8)./2.0e+2;
t54 = (t50.*x9)./2.0e+2;
t55 = (t50.*x10)./2.0e+2;
t56 = -t53;
t57 = -t54;
t58 = -t55;
B = reshape([0.0,0.0,0.0,x7.*x9.*8.125790769230769e-1+x8.*x10.*8.125790769230769e-1,x7.*x8.*(-8.125790769230769e-1)+x9.*x10.*8.125790769230769e-1,x7.^2.*4.062895384615384e-1-x8.^2.*4.062895384615384e-1-x9.^2.*4.062895384615384e-1+x10.^2.*4.062895384615384e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t56+(t27.*t51.*t59)./2.0,t52-(t18.*t51.*t59)./2.0,t55-(t19.*t51.*t59)./2.0,t57-(t20.*t51.*t59)./2.0,0.0,0.0,0.0,0.0,0.0,0.0,t57+(t27.*t51.*t60)./2.0,t58-(t18.*t51.*t60)./2.0,t52-(t19.*t51.*t60)./2.0,t53-(t20.*t51.*t60)./2.0,0.0,0.0,0.0,0.0,0.0,0.0,t58+(t27.*t51.*t61)./2.0,t54-(t18.*t51.*t61)./2.0,t56-(t19.*t51.*t61)./2.0,t52-(t20.*t51.*t61)./2.0],[10,4]);
