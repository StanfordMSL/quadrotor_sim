function conx_gate_x = conx_gate_x(in1,in2,in3,in4)
%CONX_GATE_X
%    CONX_GATE_X = CONX_GATE_X(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    13-Sep-2021 01:34:43

p01 = in2(1,:);
p02 = in2(2,:);
p03 = in2(3,:);
p11 = in3(1,:);
p12 = in3(2,:);
p13 = in3(3,:);
p21 = in4(1,:);
p22 = in4(2,:);
p23 = in4(3,:);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
t2 = -p01;
t3 = -p02;
t4 = -p03;
t5 = -p11;
t6 = -p12;
t7 = -p13;
t8 = -p21;
t9 = -p22;
t10 = -p23;
t11 = x7./5.0;
t12 = x8./5.0;
t13 = x9./5.0;
t14 = x10./5.0;
t25 = x7.*(6.0./2.5e+1);
t26 = x8.*(6.0./2.5e+1);
t27 = x9.*(6.0./2.5e+1);
t28 = x10.*(6.0./2.5e+1);
t33 = x7.*(9.0./1.0e+2);
t34 = x8.*(9.0./1.0e+2);
t35 = x9.*(9.0./1.0e+2);
t36 = x10.*(9.0./1.0e+2);
t15 = p01+t5;
t16 = p02+t6;
t17 = p03+t7;
t18 = p11+t8;
t19 = p12+t9;
t20 = p13+t10;
t21 = -t11;
t22 = -t12;
t23 = -t13;
t24 = -t14;
t29 = -t25;
t30 = -t26;
t31 = -t27;
t32 = -t28;
t40 = -t33;
t41 = -t34;
t42 = -t35;
t43 = -t36;
t47 = t13+t26+t33;
t48 = t14+t25+t34;
t49 = t11+t28+t35;
t50 = t12+t27+t36;
t37 = t18.^2;
t38 = t19.^2;
t39 = t20.^2;
t44 = t15.*t18;
t45 = t16.*t19;
t46 = t17.*t20;
t51 = t13+t26+t40;
t52 = t13+t30+t33;
t53 = t23+t26+t33;
t54 = t14+t29+t34;
t55 = t14+t25+t41;
t56 = t24+t25+t34;
t57 = t21+t28+t35;
t58 = t11+t28+t42;
t59 = t11+t32+t35;
t60 = t22+t27+t36;
t61 = t12+t31+t36;
t62 = t12+t27+t43;
t63 = t37+t38+t39;
t65 = t44+t45+t46;
t64 = 1.0./t63;
t66 = t18.*t64.*t65;
t67 = t19.*t64.*t65;
t68 = t20.*t64.*t65;
t69 = p11+t2+t66;
t70 = p12+t3+t67;
t71 = p13+t4+t68;
t72 = t47.*t69;
t73 = t48.*t69;
t74 = t49.*t69;
t75 = t50.*t69;
t76 = t47.*t70;
t77 = t48.*t70;
t78 = t49.*t70;
t79 = t50.*t70;
t80 = t47.*t71;
t81 = t48.*t71;
t82 = t49.*t71;
t83 = t50.*t71;
t84 = t51.*t69;
t85 = t52.*t69;
t86 = t53.*t69;
t87 = t54.*t69;
t88 = t55.*t69;
t89 = t56.*t69;
t90 = t57.*t69;
t91 = t58.*t69;
t92 = t59.*t69;
t93 = t60.*t69;
t94 = t61.*t69;
t95 = t62.*t69;
t96 = t51.*t70;
t97 = t52.*t70;
t98 = t53.*t70;
t99 = t54.*t70;
t100 = t55.*t70;
t101 = t56.*t70;
t102 = t57.*t70;
t103 = t58.*t70;
t104 = t59.*t70;
t105 = t60.*t70;
t106 = t61.*t70;
t107 = t62.*t70;
t108 = t51.*t71;
t109 = t52.*t71;
t110 = t53.*t71;
t111 = t54.*t71;
t112 = t55.*t71;
t113 = t56.*t71;
t114 = t57.*t71;
t115 = t58.*t71;
t116 = t59.*t71;
t117 = t60.*t71;
t118 = t61.*t71;
t119 = t62.*t71;
conx_gate_x = reshape([t69,t69,t69,t69,t69,t69,t69,t69,t70,t70,t70,t70,t70,t70,t70,t70,t71,t71,t71,t71,t71,t71,t71,t71,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t80-t91-t99,t92+t100+t110,t74-t101-t108,-t77+t90+t109,-t74+t101+t108,t77-t90-t109,-t80+t91+t99,-t92-t100-t110,-t76+t93-t111,t75-t98+t112,t94+t96-t113,-t81-t95-t97,-t94-t96+t113,t81+t95+t97,t76-t93+t111,-t75+t98-t112,t72+t105+t115,t79+t86-t116,-t82-t84+t106,t85-t107-t114,t82+t84-t106,-t85+t107+t114,-t72-t105-t115,-t79-t86+t116,t87-t103+t117,t83-t88+t104,t78+t89+t118,t73+t102-t119,-t78-t89-t118,-t73-t102+t119,-t87+t103-t117,-t83+t88-t104,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[8,17]);
