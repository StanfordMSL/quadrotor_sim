function H_r = H_r(t0,t1)
%H_R
%    H_R = H_R(T0,T1)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    23-Aug-2021 13:01:20

t3 = t0.^2;
t4 = t0.^3;
t5 = t1.^2;
t7 = t1.^3;
t8 = t0.^5;
t11 = t1.^5;
t12 = t0.^7;
t14 = t1.^7;
t20 = t0.^11;
t23 = t1.^11;
t24 = t0.^13;
t27 = t1.^13;
t32 = t0.^17;
t35 = t1.^17;
t36 = t0.^19;
t39 = t1.^19;
t6 = t3.^2;
t9 = t5.^2;
t10 = t3.^3;
t13 = t5.^3;
t16 = t4.^3;
t18 = t3.^5;
t19 = t7.^3;
t21 = t5.^5;
t26 = t3.^7;
t28 = t4.^5;
t29 = t5.^7;
t31 = t7.^5;
t41 = t3.*1.44e+3;
t42 = t5.*1.44e+3;
t43 = t4.*2.88e+3;
t44 = t7.*2.88e+3;
t47 = t8.*8.064e+3;
t48 = t11.*8.064e+3;
t57 = t12.*1.728e+4;
t58 = t14.*1.728e+4;
t59 = t8.*2.016e+4;
t60 = t11.*2.016e+4;
t65 = t12.*5.184e+4;
t66 = t14.*5.184e+4;
t80 = t20.*5.2416e+4;
t81 = t23.*5.2416e+4;
t82 = t12.*8.64e+4;
t83 = t14.*8.64e+4;
t99 = t20.*1.872e+5;
t100 = t23.*1.872e+5;
t109 = t20.*3.888e+5;
t110 = t23.*3.888e+5;
t131 = t20.*6.048e+5;
t132 = t23.*6.048e+5;
t133 = t24.*6.6528e+5;
t134 = t27.*6.6528e+5;
t138 = t24.*1.1088e+6;
t140 = t27.*1.1088e+6;
t188 = t20.*7.697454545454545e+5;
t189 = t23.*7.697454545454545e+5;
t190 = t24.*1.535261538461538e+6;
t191 = t27.*1.535261538461538e+6;
t192 = t24.*1.842313846153846e+6;
t193 = t27.*1.842313846153846e+6;
t197 = t32.*7.122409411764706e+6;
t198 = t35.*7.122409411764706e+6;
t199 = t32.*7.994541176470588e+6;
t200 = t35.*7.994541176470588e+6;
t201 = t36.*1.502132210526316e+7;
t202 = t39.*1.502132210526316e+7;
t15 = t6.^2;
t17 = t9.^2;
t22 = t6.^3;
t25 = t9.^3;
t34 = t10.^3;
t37 = t13.^3;
t38 = t6.^5;
t40 = t9.^5;
t45 = t6.*5.04e+3;
t46 = t9.*5.04e+3;
t49 = -t41;
t50 = -t43;
t52 = -t47;
t53 = t6.*1.08e+4;
t54 = t9.*1.08e+4;
t55 = t10.*1.2096e+4;
t56 = t13.*1.2096e+4;
t61 = t10.*3.36e+4;
t62 = t13.*3.36e+4;
t63 = t10.*5.04e+4;
t64 = t13.*5.04e+4;
t69 = -t57;
t70 = -t59;
t73 = t16.*3.168e+4;
t74 = t19.*3.168e+4;
t76 = t18.*4.1184e+4;
t77 = t21.*4.1184e+4;
t79 = -t65;
t87 = -t80;
t90 = -t82;
t91 = t16.*1.056e+5;
t92 = t19.*1.056e+5;
t95 = t18.*1.4256e+5;
t96 = t21.*1.4256e+5;
t101 = t16.*2.016e+5;
t102 = t19.*2.016e+5;
t105 = t16.*2.8224e+5;
t106 = t19.*2.8224e+5;
t107 = t18.*2.8512e+5;
t108 = t21.*2.8512e+5;
t111 = t18.*4.2336e+5;
t112 = t21.*4.2336e+5;
t113 = t18.*5.08032e+5;
t114 = t21.*5.08032e+5;
t122 = -t99;
t127 = -t109;
t143 = t26.*1.44144e+6;
t144 = t29.*1.44144e+6;
t145 = t26.*2.0592e+6;
t146 = t29.*2.0592e+6;
t147 = t26.*2.56608e+6;
t148 = t29.*2.56608e+6;
t149 = t28.*2.690688e+6;
t150 = t31.*2.690688e+6;
t151 = t26.*2.8512e+6;
t152 = t29.*2.8512e+6;
t153 = t28.*3.459456e+6;
t154 = t31.*3.459456e+6;
t155 = t28.*3.99168e+6;
t156 = t31.*3.99168e+6;
t163 = -t131;
t164 = -t133;
t167 = -t138;
t194 = -t188;
t195 = -t190;
t196 = -t192;
t203 = -t197;
t204 = -t199;
t205 = -t201;
t30 = t15.^2;
t33 = t17.^2;
t51 = -t45;
t67 = -t53;
t68 = -t55;
t71 = t15.*2.376e+4;
t72 = t17.*2.376e+4;
t75 = -t61;
t78 = -t63;
t85 = -t73;
t86 = -t76;
t88 = t15.*7.56e+4;
t89 = t17.*7.56e+4;
t93 = t15.*1.3608e+5;
t94 = t17.*1.3608e+5;
t97 = t15.*1.764e+5;
t98 = t17.*1.764e+5;
t103 = t22.*2.4024e+5;
t104 = t25.*2.4024e+5;
t115 = t22.*5.148e+5;
t116 = t25.*5.148e+5;
t118 = -t91;
t120 = -t95;
t123 = -t101;
t125 = -t105;
t126 = -t107;
t128 = -t111;
t129 = -t113;
t135 = t22.*8.316e+5;
t136 = t25.*8.316e+5;
t137 = t22.*1.1088e+6;
t139 = t25.*1.1088e+6;
t141 = t22.*1.27008e+6;
t142 = t25.*1.27008e+6;
t169 = -t143;
t170 = -t145;
t171 = -t147;
t172 = -t149;
t173 = -t151;
t174 = -t153;
t175 = -t155;
t179 = t34.*1.057056e+7;
t180 = t37.*1.057056e+7;
t181 = t34.*1.13256e+7;
t182 = t37.*1.13256e+7;
t183 = t38.*2.0612592e+7;
t184 = t40.*2.0612592e+7;
t206 = t42+t49;
t207 = t44+t50;
t209 = t48+t52;
t212 = t58+t69;
t213 = t60+t70;
t216 = t66+t79;
t220 = t81+t87;
t221 = t83+t90;
t227 = t100+t122;
t232 = t110+t127;
t236 = t132+t163;
t237 = t134+t164;
t240 = t140+t167;
t255 = t189+t194;
t256 = t191+t195;
t257 = t193+t196;
t258 = t198+t203;
t259 = t200+t204;
t260 = t202+t205;
t84 = -t71;
t117 = -t88;
t119 = -t93;
t121 = -t97;
t124 = -t103;
t130 = -t115;
t157 = t30.*4.540536e+6;
t158 = t33.*4.540536e+6;
t159 = t30.*5.4054e+6;
t160 = t33.*5.4054e+6;
t161 = t30.*5.8806e+6;
t162 = t33.*5.8806e+6;
t165 = -t135;
t166 = -t137;
t168 = -t141;
t185 = -t179;
t186 = -t181;
t187 = -t183;
t208 = t46+t51;
t210 = t54+t67;
t211 = t56+t68;
t214 = t62+t75;
t215 = t64+t78;
t218 = t74+t85;
t219 = t77+t86;
t223 = t92+t118;
t225 = t96+t120;
t228 = t102+t123;
t230 = t106+t125;
t231 = t108+t126;
t233 = t112+t128;
t234 = t114+t129;
t242 = t144+t169;
t243 = t146+t170;
t244 = t148+t171;
t245 = t150+t172;
t246 = t152+t173;
t247 = t154+t174;
t248 = t156+t175;
t176 = -t157;
t177 = -t159;
t178 = -t161;
t217 = t72+t84;
t222 = t89+t117;
t224 = t94+t119;
t226 = t98+t121;
t229 = t104+t124;
t235 = t116+t130;
t238 = t136+t165;
t239 = t139+t166;
t241 = t142+t168;
t252 = t180+t185;
t253 = t182+t186;
t254 = t184+t187;
t249 = t158+t176;
t250 = t160+t177;
t251 = t162+t178;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t0.*-5.76e+2+t1.*5.76e+2,t206,t207,t208,t209,t211,t212,t217,t218,t219,t220,0.0,0.0,0.0,0.0,t206,t4.*-4.8e+3+t7.*4.8e+3,t210,t213,t214,t216,t222,t223,t225,t227,t229,0.0,0.0,0.0,0.0,t207,t210,t8.*-2.592e+4+t11.*2.592e+4,t215,t221,t224,t228,t231,t232,t235,t237,0.0,0.0,0.0,0.0,t208,t213,t215,t12.*-1.008e+5+t14.*1.008e+5,t226,t230,t233,t236,t238,t240,t242,0.0,0.0,0.0,0.0,t209,t214,t221,t226];
mt2 = [t16.*-3.136e+5+t19.*3.136e+5,t234,t255,t239,t256,t243,t245,0.0,0.0,0.0,0.0,t211,t216,t224,t230,t234,t20.*(-8.313250909090909e+5)+t23.*8.313250909090909e+5,t241,t257,t244,t247,t249,0.0,0.0,0.0,0.0,t212,t222,t228,t233,t255,t241,t24.*(-1.953969230769231e+6)+t27.*1.953969230769231e+6,t246,t248,t250,t258,0.0,0.0,0.0,0.0,t217,t223,t231,t236,t239,t257,t246,t28.*-4.18176e+6+t31.*4.18176e+6,t251,t259,t252,0.0,0.0,0.0,0.0,t218,t225,t232,t238,t256,t244,t248,t251,t32.*(-8.302023529411765e+6)+t35.*8.302023529411765e+6,t253,t260,0.0,0.0,0.0,0.0,t219,t227,t235,t240,t243,t247,t250,t259,t253];
mt3 = [t36.*(-1.549818947368421e+7)+t39.*1.549818947368421e+7,t254,0.0,0.0,0.0,0.0,t220,t229,t237,t242,t245,t249,t258,t252,t260,t254,t4.^7.*-2.7483456e+7+t7.^7.*2.7483456e+7];
H_r = reshape([mt1,mt2,mt3],15,15);
