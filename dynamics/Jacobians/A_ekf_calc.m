function A_ekf = A_ekf_calc(in1,in2)
%A_EKF_CALC
%    A_EKF = A_EKF_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    23-Jul-2021 13:27:16

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
t2 = u_ses1.^2;
t3 = u_ses2.^2;
t4 = u_ses3.^2;
t5 = u_ses4.^2;
t6 = x_ses7.*2.0;
t7 = x_ses8.*2.0;
t8 = x_ses9.*2.0;
t9 = x_ses10.*2.0;
t10 = x_ses7.^2;
t11 = x_ses8.^2;
t12 = x_ses9.^2;
t13 = x_ses10.^2;
t20 = -x_ses7;
t23 = x_ses7.*x_ses9.*-2.0;
t24 = x_ses8.*x_ses9.*-2.0;
t32 = (x_ses7.*x_ses11)./2.0e+2;
t33 = (x_ses7.*x_ses12)./2.0e+2;
t34 = (x_ses8.*x_ses11)./2.0e+2;
t35 = (x_ses7.*x_ses13)./2.0e+2;
t36 = (x_ses8.*x_ses12)./2.0e+2;
t37 = (x_ses9.*x_ses11)./2.0e+2;
t38 = (x_ses8.*x_ses13)./2.0e+2;
t39 = (x_ses9.*x_ses12)./2.0e+2;
t40 = (x_ses10.*x_ses11)./2.0e+2;
t41 = (x_ses9.*x_ses13)./2.0e+2;
t42 = (x_ses10.*x_ses12)./2.0e+2;
t43 = (x_ses10.*x_ses13)./2.0e+2;
t59 = x_ses13.*6.298701298701299e-3;
t63 = (x_ses4.*x_ses7)./7.7e+7;
t64 = (x_ses4.*x_ses8)./7.7e+7;
t65 = (x_ses5.*x_ses7)./7.7e+7;
t66 = (x_ses4.*x_ses9)./7.7e+7;
t67 = (x_ses5.*x_ses8)./7.7e+7;
t68 = (x_ses6.*x_ses7)./7.7e+7;
t69 = (x_ses4.*x_ses10)./7.7e+7;
t70 = (x_ses5.*x_ses9)./7.7e+7;
t71 = (x_ses6.*x_ses8)./7.7e+7;
t72 = (x_ses5.*x_ses10)./7.7e+7;
t73 = (x_ses6.*x_ses9)./7.7e+7;
t74 = (x_ses6.*x_ses10)./7.7e+7;
t75 = (x_ses7.*x_ses10)./7.7e+7;
t76 = (x_ses8.*x_ses9)./7.7e+7;
t88 = (x_ses4.*x_ses7)./2.75e+10;
t89 = (x_ses4.*x_ses8)./2.75e+10;
t90 = (x_ses5.*x_ses7)./2.75e+10;
t91 = (x_ses4.*x_ses9)./2.75e+10;
t92 = (x_ses5.*x_ses8)./2.75e+10;
t93 = (x_ses6.*x_ses7)./2.75e+10;
t94 = (x_ses4.*x_ses10)./2.75e+10;
t95 = (x_ses5.*x_ses9)./2.75e+10;
t96 = (x_ses6.*x_ses8)./2.75e+10;
t97 = (x_ses5.*x_ses10)./2.75e+10;
t98 = (x_ses6.*x_ses9)./2.75e+10;
t99 = (x_ses7.*x_ses8)./2.75e+10;
t100 = (x_ses6.*x_ses10)./2.75e+10;
t101 = (x_ses7.*x_ses9)./2.75e+10;
t102 = (x_ses7.*x_ses10)./2.75e+10;
t103 = (x_ses8.*x_ses9)./2.75e+10;
t104 = (x_ses8.*x_ses10)./2.75e+10;
t105 = (x_ses9.*x_ses10)./2.75e+10;
t14 = t6.*x_ses8;
t15 = t6.*x_ses9;
t16 = t6.*x_ses10;
t17 = t7.*x_ses9;
t18 = t7.*x_ses10;
t19 = t8.*x_ses10;
t21 = -t8;
t22 = -t9;
t25 = -t10;
t26 = -t11;
t27 = -t13;
t28 = t6+t9;
t29 = t7+t8;
t44 = -t37;
t45 = -t38;
t46 = -t42;
t77 = t10./1.54e+8;
t78 = t11./1.54e+8;
t79 = t12./1.54e+8;
t80 = t13./1.54e+8;
t81 = -t64;
t82 = -t65;
t83 = -t70;
t84 = -t71;
t85 = -t74;
t86 = -t76;
t106 = t10./5.5e+10;
t107 = t11./5.5e+10;
t108 = t12./5.5e+10;
t109 = t13./5.5e+10;
t110 = -t88;
t111 = -t89;
t112 = -t90;
t113 = -t92;
t114 = -t93;
t115 = -t94;
t116 = -t95;
t117 = -t98;
t118 = -t100;
t119 = -t103;
t120 = -t104;
t121 = -t105;
t136 = t20+t34+t39+t43;
t150 = t2.*3.65902e-7;
t151 = t3.*3.65902e-7;
t152 = t4.*3.65902e-7;
t153 = t5.*3.65902e-7;
t30 = t6+t22;
t31 = t7+t21;
t47 = t28.*x_ses4;
t48 = t29.*x_ses4;
t49 = t28.*x_ses5;
t50 = t29.*x_ses5;
t51 = t28.*x_ses6;
t52 = t29.*x_ses6;
t87 = -t77;
t122 = -t106;
t123 = t14+t18+t19+t23;
t125 = t10+t12+t16+t17+t26+t27;
t126 = t32+t41+t46+x_ses8;
t127 = t33+t40+t45+x_ses9;
t128 = t35+t36+t44+x_ses10;
t129 = t12+t13+t16+t24+t25+t26;
t139 = abs(t136);
t143 = sign(t136);
t144 = -x_ses4.*(t10+t11-t12-t16+t27+x_ses8.*x_ses9.*2.0);
t149 = x_ses4.*(t10+t11-t12-t16+t27+x_ses8.*x_ses9.*2.0);
t154 = t69+t82+t84;
t155 = t81+t83+t85;
t53 = t30.*x_ses4;
t54 = t31.*x_ses4;
t55 = t30.*x_ses5;
t56 = t31.*x_ses5;
t57 = t30.*x_ses6;
t58 = t31.*x_ses6;
t60 = -t47;
t124 = t123.*x_ses6;
t130 = abs(t126);
t131 = abs(t127);
t132 = abs(t128);
t133 = sign(t126);
t134 = sign(t127);
t135 = sign(t128);
t137 = t125.*x_ses5;
t148 = t139.^2;
t159 = t139.*t143.*2.0;
t185 = (t139.*t143.*x_ses8)./1.0e+2;
t186 = (t139.*t143.*x_ses9)./1.0e+2;
t187 = (t139.*t143.*x_ses10)./1.0e+2;
t188 = (t139.*t143.*x_ses11)./1.0e+2;
t189 = (t139.*t143.*x_ses12)./1.0e+2;
t190 = (t139.*t143.*x_ses13)./1.0e+2;
t61 = -t56;
t62 = -t57;
t138 = t49+t53+t58;
t140 = t130.^2;
t141 = t131.^2;
t142 = t132.^2;
t146 = t52+t55+t60;
t156 = t130.*t133.*2.0;
t157 = t131.*t134.*2.0;
t158 = t132.*t135.*2.0;
t160 = (t130.*t133.*x_ses7)./1.0e+2;
t161 = (t131.*t134.*x_ses7)./1.0e+2;
t162 = (t132.*t135.*x_ses7)./1.0e+2;
t163 = (t131.*t134.*x_ses8)./1.0e+2;
t164 = (t132.*t135.*x_ses8)./1.0e+2;
t165 = (t130.*t133.*x_ses9)./1.0e+2;
t166 = (t132.*t135.*x_ses9)./1.0e+2;
t167 = (t130.*t133.*x_ses10)./1.0e+2;
t168 = (t131.*t134.*x_ses10)./1.0e+2;
t169 = (t130.*t133.*x_ses11)./1.0e+2;
t170 = (t131.*t134.*x_ses11)./1.0e+2;
t171 = (t132.*t135.*x_ses11)./1.0e+2;
t172 = (t130.*t133.*x_ses12)./1.0e+2;
t173 = (t131.*t134.*x_ses12)./1.0e+2;
t174 = (t132.*t135.*x_ses12)./1.0e+2;
t175 = (t130.*t133.*x_ses13)./1.0e+2;
t176 = (t131.*t134.*x_ses13)./1.0e+2;
t177 = (t132.*t135.*x_ses13)./1.0e+2;
t178 = -t159;
t191 = t124+t137+t149;
t145 = t48+t51+t61;
t147 = t50+t54+t62;
t179 = -t163;
t180 = -t166;
t181 = -t167;
t182 = -t171;
t183 = -t172;
t184 = -t176;
t192 = t191.^2;
t193 = t140+t141+t142+t148;
t210 = t169+t173+t177+t178;
t194 = 1.0./sqrt(t193);
t196 = t192.*5.500000000000001e-10;
t211 = t156+t174+t184+t188;
t212 = t157+t175+t182+t189;
t213 = t158+t170+t183+t190;
t214 = t160+t168+t180+t185;
t215 = t161+t164+t181+t186;
t216 = t162+t165+t179+t187;
t195 = t194.^3;
t197 = (t194.*x_ses7)./2.0e+2;
t198 = (t194.*x_ses8)./2.0e+2;
t199 = (t194.*x_ses9)./2.0e+2;
t200 = (t194.*x_ses10)./2.0e+2;
t201 = (t194.*x_ses11)./2.0e+2;
t202 = (t194.*x_ses12)./2.0e+2;
t203 = (t194.*x_ses13)./2.0e+2;
t217 = t150+t151+t152+t153+t196;
t204 = -t198;
t205 = -t199;
t206 = -t200;
t207 = -t201;
t208 = -t202;
t209 = -t203;
t218 = t217.*x_ses7.*(2.0./5.5e+1);
t219 = t217.*x_ses8.*(2.0./5.5e+1);
t220 = t217.*x_ses9.*(2.0./5.5e+1);
t221 = t217.*x_ses10.*(2.0./5.5e+1);
t222 = -t219;
et1 = t101+t120+t10.*t191.*(t10+t11-t12-t16+t27+x_ses8.*x_ses9.*2.0).*2.0e-11-t11.*t191.*(t10+t11-t12-t16+t27+x_ses8.*x_ses9.*2.0).*2.0e-11-t12.*t191.*(t10+t11-t12-t16+t27+x_ses8.*x_ses9.*2.0).*2.0e-11;
et2 = t13.*t191.*(t10+t11-t12-t16+t27+x_ses8.*x_ses9.*2.0).*2.0e-11;
et3 = t107+t108-t109+t122+t10.*t123.*t191.*2.0e-11-t11.*t123.*t191.*2.0e-11-t12.*t123.*t191.*2.0e-11;
et4 = t13.*t123.*t191.*2.0e-11+1.0;
et5 = t88-t97+t98-t220+t10.*t147.*t191.*2.0e-11-t11.*t147.*t191.*2.0e-11-t12.*t147.*t191.*2.0e-11;
et6 = t13.*t147.*t191.*2.0e-11;
mt1 = [1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./1.0e+2,0.0,0.0,-t107+t108+t109+t122+t191.*x_ses7.*x_ses9.*(t10+t11-t12-t16+t27+x_ses8.*x_ses9.*2.0).*4.0e-11+t191.*x_ses8.*x_ses10.*(t10+t11-t12-t16+t27+x_ses8.*x_ses9.*2.0).*4.0e-11+1.0];
mt2 = [-t102+t119-t191.*x_ses7.*x_ses8.*(t10+t11-t12-t16+t27+x_ses8.*x_ses9.*2.0).*4.0e-11+t191.*x_ses9.*x_ses10.*(t10+t11-t12-t16+t27+x_ses8.*x_ses9.*2.0).*4.0e-11,et1+et2,0.0,0.0,0.0,0.0,-t78+t79+t80+t87,t75+t86,x_ses7.*x_ses9.*(-7.968127490039841e-9)-(x_ses8.*x_ses10)./1.255e+8,0.0,1.0./1.0e+2,0.0];
mt3 = [t102+t119+t125.*t191.*x_ses7.*x_ses9.*4.0e-11+t125.*t191.*x_ses8.*x_ses10.*4.0e-11];
mt4 = [t107-t108+t109+t122-t125.*t191.*x_ses7.*x_ses8.*4.0e-11+t125.*t191.*x_ses9.*x_ses10.*4.0e-11+1.0];
mt5 = [-t99+t121+t10.*t125.*t191.*2.0e-11-t11.*t125.*t191.*2.0e-11-t12.*t125.*t191.*2.0e-11+t13.*t125.*t191.*2.0e-11];
mt6 = [0.0,0.0,0.0,0.0,-t75+t86,t78-t79+t80+t87,(x_ses7.*x_ses8)./1.255e+8-(x_ses9.*x_ses10)./1.255e+8,0.0,0.0,1.0./1.0e+2,-t101+t120+t123.*t191.*x_ses7.*x_ses9.*4.0e-11+t123.*t191.*x_ses8.*x_ses10.*4.0e-11];
mt7 = [t99+t121-t123.*t191.*x_ses7.*x_ses8.*4.0e-11+t123.*t191.*x_ses9.*x_ses10.*4.0e-11,et3+et4,0.0,0.0,0.0,0.0,(x_ses7.*x_ses9)./7.7e+7-(x_ses8.*x_ses10)./7.7e+7,x_ses7.*x_ses8.*(-1.298701298701299e-8)-(x_ses9.*x_ses10)./7.7e+7];
mt8 = [t10.*(-3.98406374501992e-9)+t11./2.51e+8+t12./2.51e+8-t13./2.51e+8,0.0,0.0,0.0,t97+t110+t117+t220+t138.*t191.*x_ses7.*x_ses9.*4.0e-11+t138.*t191.*x_ses8.*x_ses10.*4.0e-11];
mt9 = [t96+t112+t115+t222-t138.*t191.*x_ses7.*x_ses8.*4.0e-11+t138.*t191.*x_ses9.*x_ses10.*4.0e-11];
mt10 = [t91+t113+t114+t218+t10.*t138.*t191.*2.0e-11-t11.*t138.*t191.*2.0e-11-t12.*t138.*t191.*2.0e-11+t13.*t138.*t191.*2.0e-11];
mt11 = [t194+(t136.*t195.*t210)./2.0,t201-(t126.*t195.*t210)./2.0,t202-(t127.*t195.*t210)./2.0,t203-(t128.*t195.*t210)./2.0,-t63-t72+t73,t154,x_ses4.*x_ses9.*(-7.968127490039841e-9)+(x_ses5.*x_ses8)./1.255e+8-(x_ses6.*x_ses7)./1.255e+8,0.0,0.0,0.0];
mt12 = [t111+t116+t118+t221+t145.*t191.*x_ses7.*x_ses9.*4.0e-11+t145.*t191.*x_ses8.*x_ses10.*4.0e-11];
mt13 = [-t91+t92+t93-t218-t145.*t191.*x_ses7.*x_ses8.*4.0e-11+t145.*t191.*x_ses9.*x_ses10.*4.0e-11];
mt14 = [t96+t112+t115+t222+t10.*t145.*t191.*2.0e-11-t11.*t145.*t191.*2.0e-11-t12.*t145.*t191.*2.0e-11+t13.*t145.*t191.*2.0e-11];
mt15 = [t207+(t136.*t195.*t211)./2.0,t194-(t126.*t195.*t211)./2.0,t209-(t127.*t195.*t211)./2.0,t202-(t128.*t195.*t211)./2.0,t155,-t66+t67-t68,(x_ses5.*x_ses7)./1.255e+8-(x_ses4.*x_ses10)./1.255e+8+(x_ses6.*x_ses8)./1.255e+8,0.0,0.0,0.0,t91+t113+t114+t218+t147.*t191.*x_ses7.*x_ses9.*4.0e-11+t147.*t191.*x_ses8.*x_ses10.*4.0e-11];
mt16 = [t111+t116+t118+t221-t147.*t191.*x_ses7.*x_ses8.*4.0e-11+t147.*t191.*x_ses9.*x_ses10.*4.0e-11,et5+et6,t208+(t136.*t195.*t212)./2.0,t203-(t126.*t195.*t212)./2.0,t194-(t127.*t195.*t212)./2.0,t207-(t128.*t195.*t212)./2.0,t66-t67+t68,t155,x_ses4.*x_ses7.*(-7.968127490039841e-9)-(x_ses5.*x_ses10)./1.255e+8+(x_ses6.*x_ses9)./1.255e+8,0.0];
mt17 = [0.0,0.0,t90+t94-t96+t219+t146.*t191.*x_ses7.*x_ses9.*4.0e-11+t146.*t191.*x_ses8.*x_ses10.*4.0e-11];
mt18 = [t97+t110+t117+t220-t146.*t191.*x_ses7.*x_ses8.*4.0e-11+t146.*t191.*x_ses9.*x_ses10.*4.0e-11];
mt19 = [t111+t116+t118+t221+t10.*t146.*t191.*2.0e-11-t11.*t146.*t191.*2.0e-11-t12.*t146.*t191.*2.0e-11+t13.*t146.*t191.*2.0e-11];
mt20 = [t209+(t136.*t195.*t213)./2.0,t208-(t126.*t195.*t213)./2.0,t201-(t127.*t195.*t213)./2.0,t194-(t128.*t195.*t213)./2.0,t154,t63+t72-t73,x_ses4.*x_ses8.*(-7.968127490039841e-9)-(x_ses5.*x_ses9)./1.255e+8-(x_ses6.*x_ses10)./1.255e+8,0.0,0.0,0.0,0.0,0.0,0.0,t204+(t136.*t195.*t214)./2.0,t197-(t126.*t195.*t214)./2.0,t200-(t127.*t195.*t214)./2.0,t205-(t128.*t195.*t214)./2.0,9.999999935064935e-1,t59,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t205+(t136.*t195.*t215)./2.0,t206-(t126.*t195.*t215)./2.0,t197-(t127.*t195.*t215)./2.0];
mt21 = [t198-(t128.*t195.*t215)./2.0,-t59,9.999999935064935e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t206+(t136.*t195.*t216)./2.0,t199-(t126.*t195.*t216)./2.0,t204-(t127.*t195.*t216)./2.0,t197-(t128.*t195.*t216)./2.0,x_ses12.*(-6.298701298701299e-3),x_ses11.*6.298701298701299e-3,9.999999960159363e-1];
A_ekf = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11,mt12,mt13,mt14,mt15,mt16,mt17,mt18,mt19,mt20,mt21],13,13);
