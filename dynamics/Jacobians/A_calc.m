function A = A_calc(in1,in2)
%A_CALC
%    A = A_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    18-Aug-2021 19:02:29

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
t18 = u1.*x7.*3.149009433962264e-1;
t19 = u1.*x8.*3.149009433962264e-1;
t20 = u1.*x9.*3.149009433962264e-1;
t21 = u1.*x10.*3.149009433962264e-1;
t15 = -t6;
t16 = -t11;
t17 = -t12;
t22 = -t19;
t32 = t2+t4+t10+t14;
t23 = t3+t13+t17+x8;
t24 = t5+t9+t16+x9;
t25 = t7+t8+t15+x10;
t33 = abs(t32);
t37 = sign(t32);
t26 = abs(t23);
t27 = abs(t24);
t28 = abs(t25);
t29 = sign(t23);
t30 = sign(t24);
t31 = sign(t25);
t38 = t33.^2;
t42 = t33.*t37.*2.0;
t56 = (t33.*t37.*u2)./2.0e+2;
t57 = (t33.*t37.*u3)./2.0e+2;
t58 = (t33.*t37.*u4)./2.0e+2;
t34 = t26.^2;
t35 = t27.^2;
t36 = t28.^2;
t39 = t26.*t29.*2.0;
t40 = t27.*t30.*2.0;
t41 = t28.*t31.*2.0;
t43 = (t26.*t29.*u2)./2.0e+2;
t44 = (t27.*t30.*u2)./2.0e+2;
t45 = (t28.*t31.*u2)./2.0e+2;
t46 = (t26.*t29.*u3)./2.0e+2;
t47 = (t27.*t30.*u3)./2.0e+2;
t48 = (t28.*t31.*u3)./2.0e+2;
t49 = (t26.*t29.*u4)./2.0e+2;
t50 = (t27.*t30.*u4)./2.0e+2;
t51 = (t28.*t31.*u4)./2.0e+2;
t52 = -t42;
t53 = -t45;
t54 = -t46;
t55 = -t50;
t59 = t34+t35+t36+t38;
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
A = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,2.64e+2./2.65e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,2.64e+2./2.65e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,2.64e+2./2.65e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t20,t22,t18,t60+(t32.*t61.*t68)./2.0,t62-(t23.*t61.*t68)./2.0,t63-(t24.*t61.*t68)./2.0,t64-(t25.*t61.*t68)./2.0,0.0,0.0,0.0,t21,-t18,t22,t65+(t32.*t61.*t69)./2.0,t60-(t23.*t61.*t69)./2.0,t67-(t24.*t61.*t69)./2.0,t63-(t25.*t61.*t69)./2.0,0.0,0.0,0.0,t18,t21,-t20,t66+(t32.*t61.*t70)./2.0,t64-(t23.*t61.*t70)./2.0,t60-(t24.*t61.*t70)./2.0,t65-(t25.*t61.*t70)./2.0,0.0,0.0,0.0,t19,t20,t21,t67+(t32.*t61.*t71)./2.0,t66-(t23.*t61.*t71)./2.0,t62-(t24.*t61.*t71)./2.0,t60-(t25.*t61.*t71)./2.0],[10,10]);
