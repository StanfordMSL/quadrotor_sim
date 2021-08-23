function A_ekf = A_ekf_calc(in1,in2)
%A_EKF_CALC
%    A_EKF = A_EKF_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    23-Aug-2021 13:01:28

u_ses1 = in2(1,:);
u_ses2 = in2(2,:);
u_ses3 = in2(3,:);
u_ses4 = in2(4,:);
x_ses4 = in1(4,:);
x_ses5 = in1(5,:);
x_ses6 = in1(6,:);
x_ses7 = in1(7,:);
x_ses8 = in1(8,:);
x_ses9 = in1(9,:);
x_ses10 = in1(10,:);
x_ses11 = in1(11,:);
x_ses12 = in1(12,:);
x_ses13 = in1(13,:);
t2 = sign(u_ses1);
t3 = sign(u_ses2);
t4 = sign(u_ses3);
t5 = sign(u_ses4);
t6 = u_ses1.^2;
t7 = u_ses2.^2;
t8 = u_ses3.^2;
t9 = u_ses4.^2;
t10 = x_ses7.^2;
t11 = x_ses8.^2;
t12 = x_ses9.^2;
t13 = x_ses10.^2;
t14 = -x_ses7;
t19 = (x_ses7.*x_ses11)./4.0e+2;
t20 = (x_ses7.*x_ses12)./4.0e+2;
t21 = (x_ses8.*x_ses11)./4.0e+2;
t22 = (x_ses7.*x_ses13)./4.0e+2;
t23 = (x_ses8.*x_ses12)./4.0e+2;
t24 = (x_ses9.*x_ses11)./4.0e+2;
t25 = (x_ses8.*x_ses13)./4.0e+2;
t26 = (x_ses9.*x_ses12)./4.0e+2;
t27 = (x_ses10.*x_ses11)./4.0e+2;
t28 = (x_ses9.*x_ses13)./4.0e+2;
t29 = (x_ses10.*x_ses12)./4.0e+2;
t30 = (x_ses10.*x_ses13)./4.0e+2;
t15 = t2.*t6;
t16 = t3.*t7;
t17 = t4.*t8;
t18 = t5.*t9;
t31 = -t24;
t32 = -t25;
t33 = -t29;
t44 = t14+t21+t26+t30;
t34 = t15+t16+t17+t18;
t35 = t19+t28+t33+x_ses8;
t36 = t20+t27+t32+x_ses9;
t37 = t22+t23+t31+x_ses10;
t45 = abs(t44);
t49 = sign(t44);
t38 = abs(t35);
t39 = abs(t36);
t40 = abs(t37);
t41 = sign(t35);
t42 = sign(t36);
t43 = sign(t37);
t50 = t45.^2;
t54 = t45.*t49.*2.0;
t80 = (t45.*t49.*x_ses8)./2.0e+2;
t81 = (t45.*t49.*x_ses9)./2.0e+2;
t82 = (t45.*t49.*x_ses10)./2.0e+2;
t83 = (t45.*t49.*x_ses11)./2.0e+2;
t84 = (t45.*t49.*x_ses12)./2.0e+2;
t85 = (t45.*t49.*x_ses13)./2.0e+2;
t86 = t34.*x_ses7.*4.433962264150943e-9;
t87 = t34.*x_ses8.*4.433962264150943e-9;
t88 = t34.*x_ses9.*4.433962264150943e-9;
t89 = t34.*x_ses10.*4.433962264150943e-9;
t46 = t38.^2;
t47 = t39.^2;
t48 = t40.^2;
t51 = t38.*t41.*2.0;
t52 = t39.*t42.*2.0;
t53 = t40.*t43.*2.0;
t55 = (t38.*t41.*x_ses7)./2.0e+2;
t56 = (t39.*t42.*x_ses7)./2.0e+2;
t57 = (t40.*t43.*x_ses7)./2.0e+2;
t58 = (t39.*t42.*x_ses8)./2.0e+2;
t59 = (t40.*t43.*x_ses8)./2.0e+2;
t60 = (t38.*t41.*x_ses9)./2.0e+2;
t61 = (t40.*t43.*x_ses9)./2.0e+2;
t62 = (t38.*t41.*x_ses10)./2.0e+2;
t63 = (t39.*t42.*x_ses10)./2.0e+2;
t64 = (t38.*t41.*x_ses11)./2.0e+2;
t65 = (t39.*t42.*x_ses11)./2.0e+2;
t66 = (t40.*t43.*x_ses11)./2.0e+2;
t67 = (t38.*t41.*x_ses12)./2.0e+2;
t68 = (t39.*t42.*x_ses12)./2.0e+2;
t69 = (t40.*t43.*x_ses12)./2.0e+2;
t70 = (t38.*t41.*x_ses13)./2.0e+2;
t71 = (t39.*t42.*x_ses13)./2.0e+2;
t72 = (t40.*t43.*x_ses13)./2.0e+2;
t73 = -t54;
t90 = -t87;
t74 = -t58;
t75 = -t61;
t76 = -t62;
t77 = -t66;
t78 = -t67;
t79 = -t71;
t91 = t46+t47+t48+t50;
t107 = t64+t68+t72+t73;
t92 = 1.0./sqrt(t91);
t108 = t51+t69+t79+t83;
t109 = t52+t70+t77+t84;
t110 = t53+t65+t78+t85;
t111 = t55+t63+t75+t80;
t112 = t56+t59+t76+t81;
t113 = t57+t60+t74+t82;
t93 = t92.^3;
t94 = (t92.*x_ses7)./4.0e+2;
t95 = (t92.*x_ses8)./4.0e+2;
t96 = (t92.*x_ses9)./4.0e+2;
t97 = (t92.*x_ses10)./4.0e+2;
t98 = (t92.*x_ses11)./4.0e+2;
t99 = (t92.*x_ses12)./4.0e+2;
t100 = (t92.*x_ses13)./4.0e+2;
t101 = -t95;
t102 = -t96;
t103 = -t97;
t104 = -t98;
t105 = -t99;
t106 = -t100;
mt1 = [1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,5.29e+2./5.3e+2,0.0,0.0,0.0,0.0,0.0,0.0,t10.*(-4.504504504504505e-9)-t11./2.22e+8+t12./2.22e+8+t13./2.22e+8,(x_ses7.*x_ses10)./1.64e+8-(x_ses8.*x_ses9)./1.64e+8,x_ses7.*x_ses9.*(-3.937007874015748e-9)-(x_ses8.*x_ses10)./2.54e+8,0.0,1.0./2.0e+2,0.0,0.0,5.29e+2./5.3e+2,0.0,0.0,0.0,0.0,0.0];
mt2 = [x_ses7.*x_ses10.*(-9.009009009009009e-9)-(x_ses8.*x_ses9)./1.11e+8,t10.*(-3.048780487804878e-9)+t11./3.28e+8-t12./3.28e+8+t13./3.28e+8,(x_ses7.*x_ses8)./2.54e+8-(x_ses9.*x_ses10)./2.54e+8,0.0,0.0,1.0./2.0e+2,0.0,0.0,9.990566037735849e-1,0.0,0.0,0.0,0.0,(x_ses7.*x_ses9)./1.11e+8-(x_ses8.*x_ses10)./1.11e+8,x_ses7.*x_ses8.*(-6.097560975609756e-9)-(x_ses9.*x_ses10)./1.64e+8];
mt3 = [t10.*(-1.968503937007874e-9)+t11./5.08e+8+t12./5.08e+8-t13./5.08e+8,0.0,0.0,0.0,t88,t90,t86,t92+(t44.*t93.*t107)./2.0,t98-(t35.*t93.*t107)./2.0,t99-(t36.*t93.*t107)./2.0,t100-(t37.*t93.*t107)./2.0,x_ses4.*x_ses7.*(-9.009009009009009e-9)-(x_ses5.*x_ses10)./1.11e+8+(x_ses6.*x_ses9)./1.11e+8,x_ses5.*x_ses7.*(-6.097560975609756e-9)+(x_ses4.*x_ses10)./1.64e+8-(x_ses6.*x_ses8)./1.64e+8];
mt4 = [x_ses4.*x_ses9.*(-3.937007874015748e-9)+(x_ses5.*x_ses8)./2.54e+8-(x_ses6.*x_ses7)./2.54e+8,0.0,0.0,0.0,t89,-t86,t90,t104+(t44.*t93.*t108)./2.0,t92-(t35.*t93.*t108)./2.0,t106-(t36.*t93.*t108)./2.0,t99-(t37.*t93.*t108)./2.0,x_ses4.*x_ses8.*(-9.009009009009009e-9)-(x_ses5.*x_ses9)./1.11e+8-(x_ses6.*x_ses10)./1.11e+8,x_ses4.*x_ses9.*(-6.097560975609756e-9)+(x_ses5.*x_ses8)./1.64e+8-(x_ses6.*x_ses7)./1.64e+8];
mt5 = [(x_ses5.*x_ses7)./2.54e+8-(x_ses4.*x_ses10)./2.54e+8+(x_ses6.*x_ses8)./2.54e+8,0.0,0.0,0.0,t86,t89,-t88,t105+(t44.*t93.*t109)./2.0,t100-(t35.*t93.*t109)./2.0,t92-(t36.*t93.*t109)./2.0,t104-(t37.*t93.*t109)./2.0,(x_ses4.*x_ses9)./1.11e+8-(x_ses5.*x_ses8)./1.11e+8+(x_ses6.*x_ses7)./1.11e+8,x_ses4.*x_ses8.*(-6.097560975609756e-9)-(x_ses5.*x_ses9)./1.64e+8-(x_ses6.*x_ses10)./1.64e+8];
mt6 = [x_ses4.*x_ses7.*(-3.937007874015748e-9)-(x_ses5.*x_ses10)./2.54e+8+(x_ses6.*x_ses9)./2.54e+8,0.0,0.0,0.0,t87,t88,t89,t106+(t44.*t93.*t110)./2.0,t105-(t35.*t93.*t110)./2.0,t98-(t36.*t93.*t110)./2.0,t92-(t37.*t93.*t110)./2.0,x_ses5.*x_ses7.*(-9.009009009009009e-9)+(x_ses4.*x_ses10)./1.11e+8-(x_ses6.*x_ses8)./1.11e+8,(x_ses4.*x_ses7)./1.64e+8+(x_ses5.*x_ses10)./1.64e+8-(x_ses6.*x_ses9)./1.64e+8];
mt7 = [x_ses4.*x_ses8.*(-3.937007874015748e-9)-(x_ses5.*x_ses9)./2.54e+8-(x_ses6.*x_ses10)./2.54e+8,0.0,0.0,0.0,0.0,0.0,0.0,t101+(t44.*t93.*t111)./2.0,t94-(t35.*t93.*t111)./2.0,t97-(t36.*t93.*t111)./2.0,t102-(t37.*t93.*t111)./2.0,9.999999954954955e-1];
mt8 = [u_ses1.*(-2.395833333333334e-6)-u_ses2.*2.395833333333334e-6+u_ses3.*2.395833333333334e-6+u_ses4.*2.395833333333334e-6+x_ses13.*4.359756097560976e-3];
mt9 = [x_ses12.*(-1.043307086614173e-3),0.0,0.0,0.0,0.0,0.0,0.0,t102+(t44.*t93.*t112)./2.0,t103-(t35.*t93.*t112)./2.0,t94-(t36.*t93.*t112)./2.0,t95-(t37.*t93.*t112)./2.0];
mt10 = [u_ses1.*3.53978978978979e-6+u_ses2.*3.53978978978979e-6-u_ses3.*3.53978978978979e-6-u_ses4.*3.53978978978979e-6-x_ses13.*(3.0./7.4e+2)];
mt11 = [9.999999969512195e-1,x_ses11.*(-1.043307086614173e-3),0.0,0.0,0.0,0.0,0.0,0.0,t103+(t44.*t93.*t113)./2.0,t96-(t35.*t93.*t113)./2.0,t101-(t36.*t93.*t113)./2.0,t94-(t37.*t93.*t113)./2.0,x_ses12.*(-3.0./7.4e+2),x_ses11.*4.359756097560976e-3,9.999999980314961e-1];
A_ekf = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11],13,13);
