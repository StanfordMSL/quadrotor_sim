function A = A_calc(in1,in2)
%A_CALC
%    A = A_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    17-Aug-2021 12:55:51

u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
t2 = -x7;
t3 = (u2.*x7)./4.0e+2;
t4 = (u2.*x8)./4.0e+2;
t5 = (u3.*x7)./4.0e+2;
t6 = (u2.*x9)./4.0e+2;
t7 = (u3.*x8)./4.0e+2;
t8 = (u4.*x7)./4.0e+2;
t9 = (u2.*x10)./4.0e+2;
t10 = (u3.*x9)./4.0e+2;
t11 = (u4.*x8)./4.0e+2;
t12 = (u3.*x10)./4.0e+2;
t13 = (u4.*x9)./4.0e+2;
t14 = (u4.*x10)./4.0e+2;
t34 = u1.*x7.*1.410306666666667e-1;
t35 = u1.*x8.*1.410306666666667e-1;
t36 = u1.*x9.*1.410306666666667e-1;
t37 = u1.*x10.*1.410306666666667e-1;
t15 = -t6;
t16 = -t11;
t17 = -t12;
t27 = t2+t4+t10+t14;
t38 = -t35;
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
t42 = t28.*t32.*2.0;
t56 = (t28.*t32.*u2)./2.0e+2;
t57 = (t28.*t32.*u3)./2.0e+2;
t58 = (t28.*t32.*u4)./2.0e+2;
t29 = t21.^2;
t30 = t22.^2;
t31 = t23.^2;
t39 = t21.*t24.*2.0;
t40 = t22.*t25.*2.0;
t41 = t23.*t26.*2.0;
t43 = (t21.*t24.*u2)./2.0e+2;
t44 = (t22.*t25.*u2)./2.0e+2;
t45 = (t23.*t26.*u2)./2.0e+2;
t46 = (t21.*t24.*u3)./2.0e+2;
t47 = (t22.*t25.*u3)./2.0e+2;
t48 = (t23.*t26.*u3)./2.0e+2;
t49 = (t21.*t24.*u4)./2.0e+2;
t50 = (t22.*t25.*u4)./2.0e+2;
t51 = (t23.*t26.*u4)./2.0e+2;
t52 = -t42;
t53 = -t45;
t54 = -t46;
t55 = -t50;
t59 = t29+t30+t31+t33;
t68 = t43+t47+t51+t52;
t60 = 1.0./sqrt(t59);
t69 = t39+t48+t55+t56;
t70 = t40+t49+t53+t57;
t71 = t41+t44+t54+t58;
t61 = t60.^3;
t62 = (t60.*u2)./4.0e+2;
t63 = (t60.*u3)./4.0e+2;
t64 = (t60.*u4)./4.0e+2;
t65 = -t62;
t66 = -t63;
t67 = -t64;
A = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,9.999999999966667e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,9.999999999966667e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,9.999999999966667e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t36,t38,t34,t60+(t27.*t61.*t68)./2.0,t62-(t18.*t61.*t68)./2.0,t63-(t19.*t61.*t68)./2.0,t64-(t20.*t61.*t68)./2.0,0.0,0.0,0.0,t37,-t34,t38,t65+(t27.*t61.*t69)./2.0,t60-(t18.*t61.*t69)./2.0,t67-(t19.*t61.*t69)./2.0,t63-(t20.*t61.*t69)./2.0,0.0,0.0,0.0,t34,t37,-t36,t66+(t27.*t61.*t70)./2.0,t64-(t18.*t61.*t70)./2.0,t60-(t19.*t61.*t70)./2.0,t65-(t20.*t61.*t70)./2.0,0.0,0.0,0.0,t35,t36,t37,t67+(t27.*t61.*t71)./2.0,t66-(t18.*t61.*t71)./2.0,t62-(t19.*t61.*t71)./2.0,t60-(t20.*t61.*t71)./2.0],[10,10]);
