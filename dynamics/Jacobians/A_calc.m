function A = A_calc(in1,in2)
%A_CALC
%    A = A_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    15-Jul-2021 17:52:00

u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
x4 = in1(4,:);
x5 = in1(5,:);
x6 = in1(6,:);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
t2 = x7.^2;
t3 = x8.^2;
t4 = x9.^2;
t5 = x10.^2;
t6 = -x7;
t7 = (u2.*x7)./2.0e+2;
t8 = (u2.*x8)./2.0e+2;
t9 = (u3.*x7)./2.0e+2;
t10 = (u2.*x9)./2.0e+2;
t11 = (u3.*x8)./2.0e+2;
t12 = (u4.*x7)./2.0e+2;
t13 = (u2.*x10)./2.0e+2;
t14 = (u3.*x9)./2.0e+2;
t15 = (u4.*x8)./2.0e+2;
t16 = (u3.*x10)./2.0e+2;
t17 = (u4.*x9)./2.0e+2;
t18 = (u4.*x10)./2.0e+2;
t22 = (x4.*x7)./3.25e+10;
t23 = (x4.*x8)./3.25e+10;
t24 = (x5.*x7)./3.25e+10;
t25 = (x4.*x9)./3.25e+10;
t26 = (x5.*x8)./3.25e+10;
t27 = (x6.*x7)./3.25e+10;
t28 = (x4.*x10)./3.25e+10;
t29 = (x5.*x9)./3.25e+10;
t30 = (x6.*x8)./3.25e+10;
t31 = (x5.*x10)./3.25e+10;
t32 = (x6.*x9)./3.25e+10;
t33 = (x7.*x8)./3.25e+10;
t34 = (x6.*x10)./3.25e+10;
t35 = (x7.*x9)./3.25e+10;
t36 = (x7.*x10)./3.25e+10;
t37 = (x8.*x9)./3.25e+10;
t38 = (x8.*x10)./3.25e+10;
t39 = (x9.*x10)./3.25e+10;
t73 = u1.*x7.*8.125790769230769e-1;
t74 = u1.*x8.*8.125790769230769e-1;
t75 = u1.*x9.*8.125790769230769e-1;
t76 = u1.*x10.*8.125790769230769e-1;
t19 = -t10;
t20 = -t15;
t21 = -t16;
t40 = t2./6.5e+10;
t41 = t3./6.5e+10;
t42 = t4./6.5e+10;
t43 = t5./6.5e+10;
t44 = -t22;
t45 = -t23;
t46 = -t24;
t47 = -t26;
t48 = -t27;
t49 = -t28;
t50 = -t29;
t51 = -t32;
t52 = -t34;
t53 = -t37;
t54 = -t38;
t55 = -t39;
t66 = t6+t8+t14+t18;
t77 = -t74;
t56 = -t40;
t57 = t7+t17+t21+x8;
t58 = t9+t13+t20+x9;
t59 = t11+t12+t19+x10;
t67 = abs(t66);
t71 = sign(t66);
t98 = t25+t47+t48+t73;
t99 = t31+t44+t51+t75;
t100 = t30+t46+t49+t77;
t101 = t45+t50+t52+t76;
t60 = abs(t57);
t61 = abs(t58);
t62 = abs(t59);
t63 = sign(t57);
t64 = sign(t58);
t65 = sign(t59);
t72 = t67.^2;
t81 = t67.*t71.*2.0;
t95 = (t67.*t71.*u2)./1.0e+2;
t96 = (t67.*t71.*u3)./1.0e+2;
t97 = (t67.*t71.*u4)./1.0e+2;
t68 = t60.^2;
t69 = t61.^2;
t70 = t62.^2;
t78 = t60.*t63.*2.0;
t79 = t61.*t64.*2.0;
t80 = t62.*t65.*2.0;
t82 = (t60.*t63.*u2)./1.0e+2;
t83 = (t61.*t64.*u2)./1.0e+2;
t84 = (t62.*t65.*u2)./1.0e+2;
t85 = (t60.*t63.*u3)./1.0e+2;
t86 = (t61.*t64.*u3)./1.0e+2;
t87 = (t62.*t65.*u3)./1.0e+2;
t88 = (t60.*t63.*u4)./1.0e+2;
t89 = (t61.*t64.*u4)./1.0e+2;
t90 = (t62.*t65.*u4)./1.0e+2;
t91 = -t81;
t92 = -t84;
t93 = -t85;
t94 = -t89;
t102 = t68+t69+t70+t72;
t111 = t82+t86+t90+t91;
t103 = 1.0./sqrt(t102);
t112 = t78+t87+t94+t95;
t113 = t79+t88+t92+t96;
t114 = t80+t83+t93+t97;
t104 = t103.^3;
t105 = (t103.*u2)./2.0e+2;
t106 = (t103.*u3)./2.0e+2;
t107 = (t103.*u4)./2.0e+2;
t108 = -t105;
t109 = -t106;
t110 = -t107;
A = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./1.0e+2,0.0,0.0,-t41+t42+t43+t56+1.0,-t36+t53,t35+t54,0.0,0.0,0.0,0.0,0.0,1.0./1.0e+2,0.0,t36+t53,t41-t42+t43+t56+1.0,-t33+t55,0.0,0.0,0.0,0.0,0.0,0.0,1.0./1.0e+2,-t35+t54,t33+t55,t41+t42-t43+t56+1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t99,t100,t98,t103+(t66.*t104.*t111)./2.0,t105-(t57.*t104.*t111)./2.0,t106-(t58.*t104.*t111)./2.0,t107-(t59.*t104.*t111)./2.0,0.0,0.0,0.0,t101,-t25+t26+t27-t73,t100,t108+(t66.*t104.*t112)./2.0,t103-(t57.*t104.*t112)./2.0,t110-(t58.*t104.*t112)./2.0,t106-(t59.*t104.*t112)./2.0,0.0,0.0,0.0,t98,t101,t22-t31+t32-t75,t109+(t66.*t104.*t113)./2.0,t107-(t57.*t104.*t113)./2.0,t103-(t58.*t104.*t113)./2.0,t108-(t59.*t104.*t113)./2.0,0.0,0.0,0.0,t24+t28-t30+t74,t99,t101,t110+(t66.*t104.*t114)./2.0,t109-(t57.*t104.*t114)./2.0,t105-(t58.*t104.*t114)./2.0,t103-(t59.*t104.*t114)./2.0],[10,10]);
