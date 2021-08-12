function A_ekf = A_ekf_calc(in1,in2)
%A_EKF_CALC
%    A_EKF = A_EKF_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Aug-2021 13:03:18

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
t19 = x_ses4.*x_ses7.*(4.0./2.65e+2);
t20 = x_ses4.*x_ses8.*(4.0./2.65e+2);
t21 = x_ses5.*x_ses7.*(4.0./2.65e+2);
t22 = x_ses4.*x_ses9.*(4.0./2.65e+2);
t23 = x_ses5.*x_ses8.*(4.0./2.65e+2);
t24 = x_ses6.*x_ses7.*(4.0./2.65e+2);
t25 = x_ses4.*x_ses10.*(4.0./2.65e+2);
t26 = x_ses5.*x_ses9.*(4.0./2.65e+2);
t27 = x_ses6.*x_ses8.*(4.0./2.65e+2);
t28 = x_ses5.*x_ses10.*(4.0./2.65e+2);
t29 = x_ses6.*x_ses9.*(4.0./2.65e+2);
t30 = x_ses7.*x_ses8.*(4.0./2.65e+2);
t31 = x_ses6.*x_ses10.*(4.0./2.65e+2);
t32 = x_ses7.*x_ses9.*(4.0./2.65e+2);
t33 = x_ses7.*x_ses10.*(4.0./2.65e+2);
t34 = x_ses8.*x_ses9.*(4.0./2.65e+2);
t35 = x_ses8.*x_ses10.*(4.0./2.65e+2);
t36 = x_ses9.*x_ses10.*(4.0./2.65e+2);
t37 = (x_ses7.*x_ses11)./4.0e+2;
t38 = (x_ses7.*x_ses12)./4.0e+2;
t39 = (x_ses8.*x_ses11)./4.0e+2;
t40 = (x_ses7.*x_ses13)./4.0e+2;
t41 = (x_ses8.*x_ses12)./4.0e+2;
t42 = (x_ses9.*x_ses11)./4.0e+2;
t43 = (x_ses8.*x_ses13)./4.0e+2;
t44 = (x_ses9.*x_ses12)./4.0e+2;
t45 = (x_ses10.*x_ses11)./4.0e+2;
t46 = (x_ses9.*x_ses13)./4.0e+2;
t47 = (x_ses10.*x_ses12)./4.0e+2;
t48 = (x_ses10.*x_ses13)./4.0e+2;
t15 = t2.*t6;
t16 = t3.*t7;
t17 = t4.*t8;
t18 = t5.*t9;
t49 = t10.*(2.0./2.65e+2);
t50 = t11.*(2.0./2.65e+2);
t51 = t12.*(2.0./2.65e+2);
t52 = t13.*(2.0./2.65e+2);
t53 = -t19;
t54 = -t20;
t55 = -t21;
t56 = -t23;
t57 = -t24;
t58 = -t25;
t59 = -t26;
t60 = -t29;
t61 = -t31;
t62 = -t34;
t63 = -t35;
t64 = -t36;
t65 = -t42;
t66 = -t43;
t67 = -t47;
t79 = t14+t39+t44+t48;
t68 = -t49;
t69 = t15+t16+t17+t18;
t70 = t37+t46+t67+x_ses8;
t71 = t38+t45+t66+x_ses9;
t72 = t40+t41+t65+x_ses10;
t80 = abs(t79);
t84 = sign(t79);
t73 = abs(t70);
t74 = abs(t71);
t75 = abs(t72);
t76 = sign(t70);
t77 = sign(t71);
t78 = sign(t72);
t85 = t80.^2;
t89 = t80.*t84.*2.0;
t115 = t69.*x_ses7.*4.358490566037736e-9;
t116 = t69.*x_ses8.*4.358490566037736e-9;
t117 = t69.*x_ses9.*4.358490566037736e-9;
t118 = t69.*x_ses10.*4.358490566037736e-9;
t120 = (t80.*t84.*x_ses8)./2.0e+2;
t121 = (t80.*t84.*x_ses9)./2.0e+2;
t122 = (t80.*t84.*x_ses10)./2.0e+2;
t123 = (t80.*t84.*x_ses11)./2.0e+2;
t124 = (t80.*t84.*x_ses12)./2.0e+2;
t125 = (t80.*t84.*x_ses13)./2.0e+2;
t81 = t73.^2;
t82 = t74.^2;
t83 = t75.^2;
t86 = t73.*t76.*2.0;
t87 = t74.*t77.*2.0;
t88 = t75.*t78.*2.0;
t90 = (t73.*t76.*x_ses7)./2.0e+2;
t91 = (t74.*t77.*x_ses7)./2.0e+2;
t92 = (t75.*t78.*x_ses7)./2.0e+2;
t93 = (t74.*t77.*x_ses8)./2.0e+2;
t94 = (t75.*t78.*x_ses8)./2.0e+2;
t95 = (t73.*t76.*x_ses9)./2.0e+2;
t96 = (t75.*t78.*x_ses9)./2.0e+2;
t97 = (t73.*t76.*x_ses10)./2.0e+2;
t98 = (t74.*t77.*x_ses10)./2.0e+2;
t99 = (t73.*t76.*x_ses11)./2.0e+2;
t100 = (t74.*t77.*x_ses11)./2.0e+2;
t101 = (t75.*t78.*x_ses11)./2.0e+2;
t102 = (t73.*t76.*x_ses12)./2.0e+2;
t103 = (t74.*t77.*x_ses12)./2.0e+2;
t104 = (t75.*t78.*x_ses12)./2.0e+2;
t105 = (t73.*t76.*x_ses13)./2.0e+2;
t106 = (t74.*t77.*x_ses13)./2.0e+2;
t107 = (t75.*t78.*x_ses13)./2.0e+2;
t108 = -t89;
t119 = -t116;
t126 = t22+t56+t57+t115;
t127 = t28+t53+t60+t117;
t129 = t54+t59+t61+t118;
t109 = -t93;
t110 = -t96;
t111 = -t97;
t112 = -t101;
t113 = -t102;
t114 = -t106;
t128 = t27+t55+t58+t119;
t130 = t81+t82+t83+t85;
t146 = t99+t103+t107+t108;
t131 = 1.0./sqrt(t130);
t147 = t86+t104+t114+t123;
t148 = t87+t105+t112+t124;
t149 = t88+t100+t113+t125;
t150 = t90+t98+t110+t120;
t151 = t91+t94+t111+t121;
t152 = t92+t95+t109+t122;
t132 = t131.^3;
t133 = (t131.*x_ses7)./4.0e+2;
t134 = (t131.*x_ses8)./4.0e+2;
t135 = (t131.*x_ses9)./4.0e+2;
t136 = (t131.*x_ses10)./4.0e+2;
t137 = (t131.*x_ses11)./4.0e+2;
t138 = (t131.*x_ses12)./4.0e+2;
t139 = (t131.*x_ses13)./4.0e+2;
t140 = -t134;
t141 = -t135;
t142 = -t136;
t143 = -t137;
t144 = -t138;
t145 = -t139;
mt1 = [1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,-t50+t51+t52+t68+1.0,-t33+t62,t32+t63,0.0,0.0,0.0,0.0,t10.*(-5.0e-9)-t11./2.0e+8+t12./2.0e+8+t13./2.0e+8,(x_ses7.*x_ses10)./1.6e+8-(x_ses8.*x_ses9)./1.6e+8,x_ses7.*x_ses9.*(-5.0e-9)-(x_ses8.*x_ses10)./2.0e+8,0.0,1.0./2.0e+2,0.0,t33+t62];
mt2 = [t50-t51+t52+t68+1.0,-t30+t64,0.0,0.0,0.0,0.0,x_ses7.*x_ses10.*(-1.0e-8)-(x_ses8.*x_ses9)./1.0e+8,t10.*(-3.125e-9)+t11./3.2e+8-t12./3.2e+8+t13./3.2e+8,(x_ses7.*x_ses8)./2.0e+8-(x_ses9.*x_ses10)./2.0e+8,0.0,0.0,1.0./2.0e+2,-t32+t63,t30+t64,t50+t51-t52+t68+1.0,0.0,0.0,0.0,0.0,(x_ses7.*x_ses9)./1.0e+8-(x_ses8.*x_ses10)./1.0e+8];
mt3 = [x_ses7.*x_ses8.*(-6.25e-9)-(x_ses9.*x_ses10)./1.6e+8,t10.*(-2.5e-9)+t11./4.0e+8+t12./4.0e+8-t13./4.0e+8,0.0,0.0,0.0,t127,t128,t126,t131+(t79.*t132.*t146)./2.0,t137-(t70.*t132.*t146)./2.0,t138-(t71.*t132.*t146)./2.0,t139-(t72.*t132.*t146)./2.0,x_ses4.*x_ses7.*(-1.0e-8)-(x_ses5.*x_ses10)./1.0e+8+(x_ses6.*x_ses9)./1.0e+8];
mt4 = [x_ses5.*x_ses7.*(-6.25e-9)+(x_ses4.*x_ses10)./1.6e+8-(x_ses6.*x_ses8)./1.6e+8,x_ses4.*x_ses9.*(-5.0e-9)+(x_ses5.*x_ses8)./2.0e+8-(x_ses6.*x_ses7)./2.0e+8,0.0,0.0,0.0,t129,-t22+t23+t24-t115,t128,t143+(t79.*t132.*t147)./2.0,t131-(t70.*t132.*t147)./2.0,t145-(t71.*t132.*t147)./2.0,t138-(t72.*t132.*t147)./2.0,x_ses4.*x_ses8.*(-1.0e-8)-(x_ses5.*x_ses9)./1.0e+8-(x_ses6.*x_ses10)./1.0e+8];
mt5 = [x_ses4.*x_ses9.*(-6.25e-9)+(x_ses5.*x_ses8)./1.6e+8-(x_ses6.*x_ses7)./1.6e+8,(x_ses5.*x_ses7)./2.0e+8-(x_ses4.*x_ses10)./2.0e+8+(x_ses6.*x_ses8)./2.0e+8,0.0,0.0,0.0,t126,t129,t19-t28+t29-t117,t144+(t79.*t132.*t148)./2.0,t139-(t70.*t132.*t148)./2.0,t131-(t71.*t132.*t148)./2.0,t143-(t72.*t132.*t148)./2.0,(x_ses4.*x_ses9)./1.0e+8-(x_ses5.*x_ses8)./1.0e+8+(x_ses6.*x_ses7)./1.0e+8];
mt6 = [x_ses4.*x_ses8.*(-6.25e-9)-(x_ses5.*x_ses9)./1.6e+8-(x_ses6.*x_ses10)./1.6e+8,x_ses4.*x_ses7.*(-5.0e-9)-(x_ses5.*x_ses10)./2.0e+8+(x_ses6.*x_ses9)./2.0e+8,0.0,0.0,0.0,t21+t25-t27+t116,t127,t129,t145+(t79.*t132.*t149)./2.0,t144-(t70.*t132.*t149)./2.0,t137-(t71.*t132.*t149)./2.0,t131-(t72.*t132.*t149)./2.0,x_ses5.*x_ses7.*(-1.0e-8)+(x_ses4.*x_ses10)./1.0e+8-(x_ses6.*x_ses8)./1.0e+8];
mt7 = [(x_ses4.*x_ses7)./1.6e+8+(x_ses5.*x_ses10)./1.6e+8-(x_ses6.*x_ses9)./1.6e+8,x_ses4.*x_ses8.*(-5.0e-9)-(x_ses5.*x_ses9)./2.0e+8-(x_ses6.*x_ses10)./2.0e+8,0.0,0.0,0.0,0.0,0.0,0.0,t140+(t79.*t132.*t150)./2.0,t133-(t70.*t132.*t150)./2.0,t136-(t71.*t132.*t150)./2.0,t141-(t72.*t132.*t150)./2.0,9.99999995e-1];
mt8 = [u_ses1.*(-2.4375e-6)-u_ses2.*2.4375e-6+u_ses3.*2.4375e-6+u_ses4.*2.4375e-6+x_ses13./3.2e+2,x_ses12.*(-1.5e-3),0.0,0.0,0.0,0.0,0.0];
mt9 = [0.0,t141+(t79.*t132.*t151)./2.0,t142-(t70.*t132.*t151)./2.0,t133-(t71.*t132.*t151)./2.0,t134-(t72.*t132.*t151)./2.0];
mt10 = [u_ses1.*3.9e-6+u_ses2.*3.9e-6-u_ses3.*3.9e-6-u_ses4.*3.9e-6-x_ses13./5.0e+2,9.99999996875e-1];
mt11 = [x_ses11.*(-1.5e-3),0.0,0.0,0.0,0.0,0.0,0.0,t142+(t79.*t132.*t152)./2.0,t135-(t70.*t132.*t152)./2.0,t140-(t71.*t132.*t152)./2.0,t133-(t72.*t132.*t152)./2.0,x_ses12.*(-1.0./5.0e+2),x_ses11./3.2e+2,9.999999975e-1];
A_ekf = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11],13,13);
