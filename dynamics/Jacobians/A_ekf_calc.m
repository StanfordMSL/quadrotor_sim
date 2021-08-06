function A_ekf = A_ekf_calc(in1,in2)
%A_EKF_CALC
%    A_EKF = A_EKF_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    05-Aug-2021 17:00:55

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
t10 = x_ses7.*2.0;
t11 = x_ses8.*2.0;
t12 = x_ses9.*2.0;
t13 = x_ses10.*2.0;
t14 = x_ses7.^2;
t15 = x_ses8.^2;
t16 = x_ses9.^2;
t17 = x_ses10.^2;
t24 = -x_ses7;
t27 = x_ses7.*x_ses9.*-2.0;
t28 = x_ses8.*x_ses9.*-2.0;
t36 = (x_ses7.*x_ses11)./4.0e+2;
t37 = (x_ses7.*x_ses12)./4.0e+2;
t38 = (x_ses8.*x_ses11)./4.0e+2;
t39 = (x_ses7.*x_ses13)./4.0e+2;
t40 = (x_ses8.*x_ses12)./4.0e+2;
t41 = (x_ses9.*x_ses11)./4.0e+2;
t42 = (x_ses8.*x_ses13)./4.0e+2;
t43 = (x_ses9.*x_ses12)./4.0e+2;
t44 = (x_ses10.*x_ses11)./4.0e+2;
t45 = (x_ses9.*x_ses13)./4.0e+2;
t46 = (x_ses10.*x_ses12)./4.0e+2;
t47 = (x_ses10.*x_ses13)./4.0e+2;
t63 = x_ses13.*4.484536082474227e-3;
t67 = (x_ses4.*x_ses7)./2.91e+9;
t68 = (x_ses4.*x_ses8)./2.91e+9;
t69 = (x_ses5.*x_ses7)./2.91e+9;
t70 = (x_ses4.*x_ses9)./2.91e+9;
t71 = (x_ses5.*x_ses8)./2.91e+9;
t72 = (x_ses6.*x_ses7)./2.91e+9;
t73 = (x_ses4.*x_ses10)./2.91e+9;
t74 = (x_ses5.*x_ses9)./2.91e+9;
t75 = (x_ses6.*x_ses8)./2.91e+9;
t76 = (x_ses5.*x_ses10)./2.91e+9;
t77 = (x_ses6.*x_ses9)./2.91e+9;
t78 = (x_ses6.*x_ses10)./2.91e+9;
t79 = (x_ses7.*x_ses10)./2.91e+9;
t80 = (x_ses8.*x_ses9)./2.91e+9;
t91 = (x_ses4.*x_ses7)./1.5e+11;
t92 = (x_ses4.*x_ses8)./1.5e+11;
t93 = (x_ses5.*x_ses7)./1.5e+11;
t94 = (x_ses4.*x_ses9)./1.5e+11;
t95 = (x_ses5.*x_ses8)./1.5e+11;
t96 = (x_ses6.*x_ses7)./1.5e+11;
t97 = (x_ses4.*x_ses10)./1.5e+11;
t98 = (x_ses5.*x_ses9)./1.5e+11;
t99 = (x_ses6.*x_ses8)./1.5e+11;
t100 = (x_ses5.*x_ses10)./1.5e+11;
t101 = (x_ses6.*x_ses9)./1.5e+11;
t102 = (x_ses7.*x_ses8)./1.5e+11;
t103 = (x_ses6.*x_ses10)./1.5e+11;
t104 = (x_ses7.*x_ses9)./1.5e+11;
t105 = (x_ses7.*x_ses10)./1.5e+11;
t106 = (x_ses8.*x_ses9)./1.5e+11;
t107 = (x_ses8.*x_ses10)./1.5e+11;
t108 = (x_ses9.*x_ses10)./1.5e+11;
t18 = t10.*x_ses8;
t19 = t10.*x_ses9;
t20 = t10.*x_ses10;
t21 = t11.*x_ses9;
t22 = t11.*x_ses10;
t23 = t12.*x_ses10;
t25 = -t12;
t26 = -t13;
t29 = -t14;
t30 = -t15;
t31 = -t17;
t32 = t10+t13;
t33 = t11+t12;
t48 = -t41;
t49 = -t42;
t50 = -t46;
t81 = t14./5.82e+9;
t82 = t15./5.82e+9;
t83 = t16./5.82e+9;
t84 = t17./5.82e+9;
t85 = -t68;
t86 = -t69;
t87 = -t74;
t88 = -t75;
t89 = -t78;
t90 = -t80;
t110 = t14./3.0e+11;
t111 = t15./3.0e+11;
t112 = t16./3.0e+11;
t113 = t17./3.0e+11;
t114 = -t91;
t115 = -t92;
t116 = -t93;
t117 = -t95;
t118 = -t96;
t119 = -t97;
t120 = -t98;
t121 = -t101;
t122 = -t103;
t123 = -t106;
t124 = -t107;
t125 = -t108;
t140 = t24+t38+t43+t47;
t154 = t2.*t6.*5.84e-6;
t155 = t3.*t7.*5.84e-6;
t156 = t4.*t8.*5.84e-6;
t157 = t5.*t9.*5.84e-6;
t34 = t10+t26;
t35 = t11+t25;
t51 = t32.*x_ses4;
t52 = t33.*x_ses4;
t53 = t32.*x_ses5;
t54 = t33.*x_ses5;
t55 = t32.*x_ses6;
t56 = t33.*x_ses6;
t109 = -t81;
t126 = t18+t22+t23+t27;
t127 = -t110;
t129 = t14+t16+t20+t21+t30+t31;
t130 = t36+t45+t50+x_ses8;
t131 = t37+t44+t49+x_ses9;
t132 = t39+t40+t48+x_ses10;
t133 = t16+t17+t20+t28+t29+t30;
t143 = abs(t140);
t147 = sign(t140);
t148 = -x_ses4.*(t14+t15-t16-t20+t31+x_ses8.*x_ses9.*2.0);
t153 = x_ses4.*(t14+t15-t16-t20+t31+x_ses8.*x_ses9.*2.0);
t158 = t73+t86+t88;
t159 = t85+t87+t89;
t57 = t34.*x_ses4;
t58 = t35.*x_ses4;
t59 = t34.*x_ses5;
t60 = t35.*x_ses5;
t61 = t34.*x_ses6;
t62 = t35.*x_ses6;
t64 = -t51;
t128 = t126.*x_ses6;
t134 = abs(t130);
t135 = abs(t131);
t136 = abs(t132);
t137 = sign(t130);
t138 = sign(t131);
t139 = sign(t132);
t141 = t129.*x_ses5;
t152 = t143.^2;
t163 = t143.*t147.*2.0;
t189 = (t143.*t147.*x_ses8)./2.0e+2;
t190 = (t143.*t147.*x_ses9)./2.0e+2;
t191 = (t143.*t147.*x_ses10)./2.0e+2;
t192 = (t143.*t147.*x_ses11)./2.0e+2;
t193 = (t143.*t147.*x_ses12)./2.0e+2;
t194 = (t143.*t147.*x_ses13)./2.0e+2;
t65 = -t60;
t66 = -t61;
t142 = t53+t57+t62;
t144 = t134.^2;
t145 = t135.^2;
t146 = t136.^2;
t150 = t56+t59+t64;
t160 = t134.*t137.*2.0;
t161 = t135.*t138.*2.0;
t162 = t136.*t139.*2.0;
t164 = (t134.*t137.*x_ses7)./2.0e+2;
t165 = (t135.*t138.*x_ses7)./2.0e+2;
t166 = (t136.*t139.*x_ses7)./2.0e+2;
t167 = (t135.*t138.*x_ses8)./2.0e+2;
t168 = (t136.*t139.*x_ses8)./2.0e+2;
t169 = (t134.*t137.*x_ses9)./2.0e+2;
t170 = (t136.*t139.*x_ses9)./2.0e+2;
t171 = (t134.*t137.*x_ses10)./2.0e+2;
t172 = (t135.*t138.*x_ses10)./2.0e+2;
t173 = (t134.*t137.*x_ses11)./2.0e+2;
t174 = (t135.*t138.*x_ses11)./2.0e+2;
t175 = (t136.*t139.*x_ses11)./2.0e+2;
t176 = (t134.*t137.*x_ses12)./2.0e+2;
t177 = (t135.*t138.*x_ses12)./2.0e+2;
t178 = (t136.*t139.*x_ses12)./2.0e+2;
t179 = (t134.*t137.*x_ses13)./2.0e+2;
t180 = (t135.*t138.*x_ses13)./2.0e+2;
t181 = (t136.*t139.*x_ses13)./2.0e+2;
t182 = -t163;
t195 = t128+t141+t153;
t149 = t52+t55+t65;
t151 = t54+t58+t66;
t183 = -t167;
t184 = -t170;
t185 = -t171;
t186 = -t175;
t187 = -t176;
t188 = -t180;
t196 = t195.^2;
t197 = t144+t145+t146+t152;
t214 = t173+t177+t181+t182;
t198 = t196.*1.5e-9;
t199 = 1.0./sqrt(t197);
t215 = t160+t178+t188+t192;
t216 = t161+t179+t186+t193;
t217 = t162+t174+t187+t194;
t218 = t164+t172+t184+t189;
t219 = t165+t168+t185+t190;
t220 = t166+t169+t183+t191;
t200 = t199.^3;
t201 = (t199.*x_ses7)./4.0e+2;
t202 = (t199.*x_ses8)./4.0e+2;
t203 = (t199.*x_ses9)./4.0e+2;
t204 = (t199.*x_ses10)./4.0e+2;
t205 = (t199.*x_ses11)./4.0e+2;
t206 = (t199.*x_ses12)./4.0e+2;
t207 = (t199.*x_ses13)./4.0e+2;
t221 = t154+t155+t156+t157+t198;
t208 = -t202;
t209 = -t203;
t210 = -t204;
t211 = -t205;
t212 = -t206;
t213 = -t207;
t222 = (t221.*x_ses7)./1.5e+2;
t223 = (t221.*x_ses8)./1.5e+2;
t224 = (t221.*x_ses9)./1.5e+2;
t225 = (t221.*x_ses10)./1.5e+2;
t226 = -t223;
A_ekf = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,-t111+t112+t113+t127+t195.*x_ses7.*x_ses9.*(t14+t15-t16-t20+t31+x_ses8.*x_ses9.*2.0).*2.0e-11+t195.*x_ses8.*x_ses10.*(t14+t15-t16-t20+t31+x_ses8.*x_ses9.*2.0).*2.0e-11+1.0,-t105+t123-t195.*x_ses7.*x_ses8.*(t14+t15-t16-t20+t31+x_ses8.*x_ses9.*2.0).*2.0e-11+t195.*x_ses9.*x_ses10.*(t14+t15-t16-t20+t31+x_ses8.*x_ses9.*2.0).*2.0e-11,t104+t124+t14.*t195.*(t14+t15-t16-t20+t31+x_ses8.*x_ses9.*2.0).*1.0e-11-t15.*t195.*(t14+t15-t16-t20+t31+x_ses8.*x_ses9.*2.0).*1.0e-11-t16.*t195.*(t14+t15-t16-t20+t31+x_ses8.*x_ses9.*2.0).*1.0e-11+t17.*t195.*(t14+t15-t16-t20+t31+x_ses8.*x_ses9.*2.0).*1.0e-11,0.0,0.0,0.0,0.0,-t82+t83+t84+t109,t79+t90,x_ses7.*x_ses9.*(-1.811594202898551e-10)-(x_ses8.*x_ses10)./5.52e+9,0.0,1.0./2.0e+2,0.0,t105+t123+t129.*t195.*x_ses7.*x_ses9.*2.0e-11+t129.*t195.*x_ses8.*x_ses10.*2.0e-11,t111-t112+t113+t127-t129.*t195.*x_ses7.*x_ses8.*2.0e-11+t129.*t195.*x_ses9.*x_ses10.*2.0e-11+1.0,-t102+t125+t14.*t129.*t195.*1.0e-11-t15.*t129.*t195.*1.0e-11-t16.*t129.*t195.*1.0e-11+t17.*t129.*t195.*1.0e-11,0.0,0.0,0.0,0.0,-t79+t90,t82-t83+t84+t109,(x_ses7.*x_ses8)./5.52e+9-(x_ses9.*x_ses10)./5.52e+9,0.0,0.0,1.0./2.0e+2,-t104+t124+t126.*t195.*x_ses7.*x_ses9.*2.0e-11+t126.*t195.*x_ses8.*x_ses10.*2.0e-11,t102+t125-t126.*t195.*x_ses7.*x_ses8.*2.0e-11+t126.*t195.*x_ses9.*x_ses10.*2.0e-11,t111+t112-t113+t127+t14.*t126.*t195.*1.0e-11-t15.*t126.*t195.*1.0e-11-t16.*t126.*t195.*1.0e-11+t17.*t126.*t195.*1.0e-11+1.0,0.0,0.0,0.0,0.0,(x_ses7.*x_ses9)./2.91e+9-(x_ses8.*x_ses10)./2.91e+9,x_ses7.*x_ses8.*(-3.436426116838488e-10)-(x_ses9.*x_ses10)./2.91e+9,t14.*(-9.057971014492754e-11)+t15./1.104e+10+t16./1.104e+10-t17./1.104e+10,0.0,0.0,0.0,t100+t114+t121+t224+t142.*t195.*x_ses7.*x_ses9.*2.0e-11+t142.*t195.*x_ses8.*x_ses10.*2.0e-11,t99+t116+t119+t226-t142.*t195.*x_ses7.*x_ses8.*2.0e-11+t142.*t195.*x_ses9.*x_ses10.*2.0e-11,t94+t117+t118+t222+t14.*t142.*t195.*1.0e-11-t15.*t142.*t195.*1.0e-11-t16.*t142.*t195.*1.0e-11+t17.*t142.*t195.*1.0e-11,t199+(t140.*t200.*t214)./2.0,t205-(t130.*t200.*t214)./2.0,t206-(t131.*t200.*t214)./2.0,t207-(t132.*t200.*t214)./2.0,-t67-t76+t77,t158,x_ses4.*x_ses9.*(-1.811594202898551e-10)+(x_ses5.*x_ses8)./5.52e+9-(x_ses6.*x_ses7)./5.52e+9,0.0,0.0,0.0,t115+t120+t122+t225+t149.*t195.*x_ses7.*x_ses9.*2.0e-11+t149.*t195.*x_ses8.*x_ses10.*2.0e-11,-t94+t95+t96-t222-t149.*t195.*x_ses7.*x_ses8.*2.0e-11+t149.*t195.*x_ses9.*x_ses10.*2.0e-11,t99+t116+t119+t226+t14.*t149.*t195.*1.0e-11-t15.*t149.*t195.*1.0e-11-t16.*t149.*t195.*1.0e-11+t17.*t149.*t195.*1.0e-11,t211+(t140.*t200.*t215)./2.0,t199-(t130.*t200.*t215)./2.0,t213-(t131.*t200.*t215)./2.0,t206-(t132.*t200.*t215)./2.0,t159,-t70+t71-t72,(x_ses5.*x_ses7)./5.52e+9-(x_ses4.*x_ses10)./5.52e+9+(x_ses6.*x_ses8)./5.52e+9,0.0,0.0,0.0,t94+t117+t118+t222+t151.*t195.*x_ses7.*x_ses9.*2.0e-11+t151.*t195.*x_ses8.*x_ses10.*2.0e-11,t115+t120+t122+t225-t151.*t195.*x_ses7.*x_ses8.*2.0e-11+t151.*t195.*x_ses9.*x_ses10.*2.0e-11,t91-t100+t101-t224+t14.*t151.*t195.*1.0e-11-t15.*t151.*t195.*1.0e-11-t16.*t151.*t195.*1.0e-11+t17.*t151.*t195.*1.0e-11,t212+(t140.*t200.*t216)./2.0,t207-(t130.*t200.*t216)./2.0,t199-(t131.*t200.*t216)./2.0,t211-(t132.*t200.*t216)./2.0,t70-t71+t72,t159,x_ses4.*x_ses7.*(-1.811594202898551e-10)-(x_ses5.*x_ses10)./5.52e+9+(x_ses6.*x_ses9)./5.52e+9,0.0,0.0,0.0,t93+t97-t99+t223+t150.*t195.*x_ses7.*x_ses9.*2.0e-11+t150.*t195.*x_ses8.*x_ses10.*2.0e-11,t100+t114+t121+t224-t150.*t195.*x_ses7.*x_ses8.*2.0e-11+t150.*t195.*x_ses9.*x_ses10.*2.0e-11,t115+t120+t122+t225+t14.*t150.*t195.*1.0e-11-t15.*t150.*t195.*1.0e-11-t16.*t150.*t195.*1.0e-11+t17.*t150.*t195.*1.0e-11,t213+(t140.*t200.*t217)./2.0,t212-(t130.*t200.*t217)./2.0,t205-(t131.*t200.*t217)./2.0,t199-(t132.*t200.*t217)./2.0,t158,t67+t76-t77,x_ses4.*x_ses8.*(-1.811594202898551e-10)-(x_ses5.*x_ses9)./5.52e+9-(x_ses6.*x_ses10)./5.52e+9,0.0,0.0,0.0,0.0,0.0,0.0,t208+(t140.*t200.*t218)./2.0,t201-(t130.*t200.*t218)./2.0,t204-(t131.*t200.*t218)./2.0,t209-(t132.*t200.*t218)./2.0,9.999999998281787e-1,t63,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t209+(t140.*t200.*t219)./2.0,t210-(t130.*t200.*t219)./2.0,t201-(t131.*t200.*t219)./2.0,t202-(t132.*t200.*t219)./2.0,-t63,9.999999998281787e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t210+(t140.*t200.*t220)./2.0,t203-(t130.*t200.*t220)./2.0,t208-(t131.*t200.*t220)./2.0,t201-(t132.*t200.*t220)./2.0,x_ses12.*(-4.484536082474227e-3),x_ses11.*4.484536082474227e-3,9.999999999094203e-1],[13,13]);
