function H_psi = QP_H_psi(t0,t1)
%QP_H_PSI
%    H_PSI = QP_H_PSI(T0,T1)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    05-Mar-2021 16:14:26

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
t44 = t0.^23;
t47 = t1.^23;
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
t40 = t4.^7;
t42 = t3.^11;
t43 = t7.^7;
t45 = t5.^11;
t49 = t3.*6.0;
t50 = t5.*6.0;
t52 = t4.*8.0;
t53 = t7.*8.0;
t56 = t8.*1.2e+1;
t57 = t11.*1.2e+1;
t62 = t12.*1.6e+1;
t63 = t14.*1.6e+1;
t64 = t8.*2.4e+1;
t65 = t11.*2.4e+1;
t68 = t12.*3.6e+1;
t69 = t14.*3.6e+1;
t85 = t20.*2.4e+1;
t87 = t23.*2.4e+1;
t90 = t24.*2.8e+1;
t91 = t27.*2.8e+1;
t101 = t20.*6.0e+1;
t103 = t23.*6.0e+1;
t119 = t24.*7.2e+1;
t120 = t27.*7.2e+1;
t167 = t12.*(3.6e+2./7.0);
t168 = t14.*(3.6e+2./7.0);
t232 = t20.*9.818181818181818e+1;
t233 = t23.*9.818181818181818e+1;
t236 = t20.*1.309090909090909e+2;
t237 = t23.*1.309090909090909e+2;
t239 = t24.*1.218461538461538e+2;
t240 = t27.*1.218461538461538e+2;
t242 = t20.*1.527272727272727e+2;
t243 = t23.*1.527272727272727e+2;
t244 = t24.*1.692307692307692e+2;
t245 = t27.*1.692307692307692e+2;
t246 = t24.*2.076923076923077e+2;
t247 = t27.*2.076923076923077e+2;
t248 = t24.*2.326153846153846e+2;
t249 = t27.*2.326153846153846e+2;
t251 = t32.*3.211764705882353e+2;
t252 = t35.*3.211764705882353e+2;
t254 = t32.*3.854117647058824e+2;
t255 = t35.*3.854117647058824e+2;
t256 = t32.*4.348235294117647e+2;
t257 = t35.*4.348235294117647e+2;
t258 = t32.*4.658823529411765e+2;
t259 = t35.*4.658823529411765e+2;
t272 = t36.*5.364210526315789e+2;
t273 = t39.*5.364210526315789e+2;
t274 = t36.*5.911578947368421e+2;
t275 = t39.*5.911578947368421e+2;
t276 = t36.*6.252631578947368e+2;
t277 = t39.*6.252631578947368e+2;
t278 = t44.*1.044521739130435e+3;
t279 = t47.*1.044521739130435e+3;
t15 = t6.^2;
t17 = t9.^2;
t22 = t6.^3;
t25 = t9.^3;
t34 = t10.^3;
t37 = t13.^3;
t38 = t6.^5;
t41 = t9.^5;
t51 = -t49;
t54 = t6.*1.0e+1;
t55 = t9.*1.0e+1;
t58 = t10.*1.4e+1;
t59 = t13.*1.4e+1;
t60 = t6.*1.8e+1;
t61 = t9.*1.8e+1;
t66 = t10.*3.0e+1;
t67 = t13.*3.0e+1;
t70 = t10.*4.0e+1;
t71 = t13.*4.0e+1;
t72 = -t52;
t74 = -t56;
t77 = -t62;
t80 = -t64;
t81 = t16.*2.0e+1;
t82 = t19.*2.0e+1;
t83 = t18.*2.2e+1;
t84 = t21.*2.2e+1;
t92 = -t68;
t96 = t16.*4.8e+1;
t97 = t19.*4.8e+1;
t98 = t18.*5.4e+1;
t99 = t21.*5.4e+1;
t107 = -t85;
t109 = -t90;
t114 = -t101;
t121 = t26.*7.8e+1;
t122 = t29.*7.8e+1;
t123 = t18.*1.12e+2;
t125 = t21.*1.12e+2;
t127 = t18.*1.26e+2;
t128 = t21.*1.26e+2;
t135 = t28.*2.08e+2;
t136 = t31.*2.08e+2;
t137 = t28.*2.64e+2;
t138 = t31.*2.64e+2;
t139 = t26.*2.7e+2;
t140 = t29.*2.7e+2;
t141 = t26.*2.88e+2;
t142 = t29.*2.88e+2;
t143 = t28.*3.08e+2;
t144 = t31.*3.08e+2;
t145 = t28.*3.36e+2;
t146 = t31.*3.36e+2;
t153 = -t119;
t179 = t40.*7.8e+2;
t180 = t43.*7.8e+2;
t181 = t42.*9.1e+2;
t182 = t45.*9.1e+2;
t183 = t42.*9.36e+2;
t184 = t45.*9.36e+2;
t187 = t16.*(2.24e+2./3.0);
t188 = t19.*(2.24e+2./3.0);
t189 = t16.*(2.8e+2./3.0);
t190 = t19.*(2.8e+2./3.0);
t191 = -t167;
t192 = t18.*(4.32e+2./5.0);
t193 = t21.*(4.32e+2./5.0);
t212 = t28.*(7.28e+2./5.0);
t213 = t31.*(7.28e+2./5.0);
t214 = t26.*(9.36e+2./7.0);
t215 = t29.*(9.36e+2./7.0);
t218 = t26.*1.885714285714286e+2;
t219 = t29.*1.885714285714286e+2;
t222 = t26.*2.357142857142857e+2;
t223 = t29.*2.357142857142857e+2;
t226 = t40.*8.171428571428571e+2;
t227 = t43.*8.171428571428571e+2;
t261 = -t232;
t262 = -t236;
t263 = -t239;
t264 = -t242;
t265 = -t244;
t266 = -t246;
t267 = -t248;
t268 = -t251;
t269 = -t254;
t270 = -t256;
t271 = -t258;
t280 = -t272;
t281 = -t274;
t282 = -t276;
t283 = -t278;
t30 = t15.^2;
t33 = t17.^2;
t46 = t15.^3;
t48 = t17.^3;
t73 = -t54;
t75 = -t58;
t76 = -t60;
t78 = t15.*1.8e+1;
t79 = t17.*1.8e+1;
t86 = -t66;
t88 = t22.*2.6e+1;
t89 = t25.*2.6e+1;
t93 = -t70;
t94 = t15.*4.2e+1;
t95 = t17.*4.2e+1;
t100 = t15.*6.3e+1;
t102 = t17.*6.3e+1;
t105 = -t81;
t106 = -t83;
t111 = -t96;
t112 = -t98;
t115 = t22.*6.6e+1;
t116 = t25.*6.6e+1;
t117 = t15.*7.5e+1;
t118 = t17.*7.5e+1;
t124 = t22.*1.1e+2;
t126 = t25.*1.1e+2;
t129 = t22.*1.5e+2;
t130 = t25.*1.5e+2;
t131 = t22.*1.8e+2;
t132 = t25.*1.8e+2;
t133 = t22.*1.96e+2;
t134 = t25.*1.96e+2;
t154 = -t121;
t155 = -t123;
t157 = -t127;
t161 = -t135;
t162 = -t137;
t163 = -t139;
t164 = -t141;
t165 = -t143;
t166 = -t145;
t171 = t34.*5.28e+2;
t172 = t37.*5.28e+2;
t173 = t34.*5.5e+2;
t174 = t37.*5.5e+2;
t175 = t38.*7.02e+2;
t176 = t41.*7.02e+2;
t177 = t38.*7.26e+2;
t178 = t41.*7.26e+2;
t200 = -t179;
t201 = -t181;
t202 = -t183;
t204 = -t187;
t205 = -t189;
t206 = -t192;
t216 = t34.*4.246666666666667e+2;
t217 = t37.*4.246666666666667e+2;
t220 = t34.*4.853333333333333e+2;
t221 = t37.*4.853333333333333e+2;
t224 = t38.*6.552e+2;
t225 = t41.*6.552e+2;
t230 = -t212;
t231 = -t214;
t235 = -t218;
t241 = -t222;
t253 = -t226;
t260 = t50+t51;
t284 = t53+t72;
t286 = t57+t74;
t289 = t63+t77;
t290 = t65+t80;
t292 = t69+t92;
t297 = t87+t107;
t299 = t91+t109;
t304 = t103+t114;
t307 = t120+t153;
t323 = t168+t191;
t346 = t233+t261;
t347 = t237+t262;
t348 = t240+t263;
t349 = t243+t264;
t350 = t245+t265;
t351 = t247+t266;
t352 = t249+t267;
t353 = t252+t268;
t354 = t255+t269;
t355 = t257+t270;
t356 = t259+t271;
t357 = t273+t280;
t358 = t275+t281;
t359 = t277+t282;
t360 = t279+t283;
t104 = -t78;
t108 = -t88;
t110 = -t94;
t113 = -t100;
t147 = t30.*3.85e+2;
t148 = t33.*3.85e+2;
t149 = t30.*4.05e+2;
t150 = t33.*4.05e+2;
t151 = -t115;
t152 = -t117;
t156 = -t124;
t158 = -t129;
t159 = -t131;
t160 = -t133;
t185 = t46.*1.183e+3;
t186 = t48.*1.183e+3;
t194 = t30.*(4.55e+2./2.0);
t195 = t33.*(4.55e+2./2.0);
t196 = -t171;
t197 = -t173;
t198 = -t175;
t199 = -t177;
t208 = t30.*(5.85e+2./2.0);
t209 = t33.*(5.85e+2./2.0);
t210 = t30.*(6.93e+2./2.0);
t211 = t33.*(6.93e+2./2.0);
t234 = -t216;
t238 = -t220;
t250 = -t224;
t285 = t55+t73;
t287 = t59+t75;
t288 = t61+t76;
t291 = t67+t86;
t293 = t71+t93;
t295 = t82+t105;
t296 = t84+t106;
t301 = t97+t111;
t302 = t99+t112;
t308 = t122+t154;
t309 = t125+t155;
t311 = t128+t157;
t315 = t136+t161;
t316 = t138+t162;
t317 = t140+t163;
t318 = t142+t164;
t319 = t144+t165;
t320 = t146+t166;
t328 = t180+t200;
t329 = t182+t201;
t330 = t184+t202;
t332 = t188+t204;
t333 = t190+t205;
t334 = t193+t206;
t338 = t213+t230;
t339 = t215+t231;
t341 = t219+t235;
t343 = t223+t241;
t345 = t227+t253;
t169 = -t147;
t170 = -t149;
t203 = -t185;
t207 = -t194;
t228 = -t208;
t229 = -t210;
t294 = t79+t104;
t298 = t89+t108;
t300 = t95+t110;
t303 = t102+t113;
t305 = t116+t151;
t306 = t118+t152;
t310 = t126+t156;
t312 = t130+t158;
t313 = t132+t159;
t314 = t134+t160;
t324 = t172+t196;
t325 = t174+t197;
t326 = t176+t198;
t327 = t178+t199;
t340 = t217+t234;
t342 = t221+t238;
t344 = t225+t250;
t321 = t148+t169;
t322 = t150+t170;
t331 = t186+t203;
t335 = t195+t207;
t336 = t209+t228;
t337 = t211+t229;
H_psi = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t0.*-4.0+t1.*4.0,t260,t284,t285,t286,t287,t289,t294,t295,t296,t297,t298,t299,0.0,0.0,t260,t4.*-1.2e+1+t7.*1.2e+1,t288,t290,t291,t292,t300,t301,t302,t304,t305,t307,t308,0.0,0.0,t284,t288,t8.*(-1.44e+2./5.0)+t11.*(1.44e+2./5.0),t293,t323,t303,t332,t334,t346,t310,t348,t339,t338,0.0,0.0,t285,t290,t293,t12.*(-4.0e+2./7.0)+t14.*(4.0e+2./7.0),t306,t333,t309,t347,t312,t350,t341,t315,t335,0.0,0.0,t286,t291,t323,t306,t16.*-1.0e+2+t19.*1.0e+2,t311,t349,t313,t351,t343,t316,t336,t353,0.0,0.0,t287,t292,t303,t333,t311,t20.*(-1.603636363636364e+2)+t23.*1.603636363636364e+2,t314,t352,t317,t319,t337,t354,t340,0.0,0.0,t289,t300,t332,t309,t349,t314,t24.*(-2.412307692307692e+2)+t27.*2.412307692307692e+2,t318,t320,t321,t355,t342,t357,0.0,0.0,t294,t301,t334,t347,t313,t352,t318,t28.*(-3.456e+2)+t31.*3.456e+2,t322,t356,t324,t358,t344,0.0,0.0,t295,t302,t346,t312,t351,t317,t320,t322,t32.*(-4.764705882352941e+2)+t35.*4.764705882352941e+2,t325,t359,t326,t328,0.0,0.0,t296,t304,t310,t350,t343,t319,t321,t356,t325,t36.*(-6.368421052631579e+2)+t39.*6.368421052631579e+2,t327,t345,t329,0.0,0.0,t297,t305,t348,t341,t316,t337,t355,t324,t359,t327,t40.*(-8.297142857142857e+2)+t43.*8.297142857142857e+2,t330,t360,0.0,0.0,t298,t307,t339,t315,t336,t354,t342,t358,t326,t345,t330,t44.*(-1.058086956521739e+3)+t47.*1.058086956521739e+3,t331,0.0,0.0,t299,t308,t338,t335,t353,t340,t357,t344,t328,t329,t360,t331,t8.^5.*(-1.32496e+3)+t11.^5.*1.32496e+3],[15,15]);
