function conx = conx(in1,in2)
%CONX
%    CONX = CONX(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    29-Aug-2021 18:55:37

p_box1_1 = in2(1);
p_box1_2 = in2(4);
p_box1_4 = in2(10);
p_box2_1 = in2(2);
p_box2_2 = in2(5);
p_box2_4 = in2(11);
p_box3_1 = in2(3);
p_box3_2 = in2(6);
p_box3_4 = in2(12);
x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
t2 = x7.^2;
t3 = x8.^2;
t4 = x9.^2;
t5 = x10.^2;
t6 = -p_box1_2;
t7 = -p_box1_4;
t8 = -p_box2_1;
t9 = -p_box2_2;
t10 = -p_box2_4;
t11 = -p_box3_1;
t12 = -p_box3_2;
t13 = -p_box3_4;
t14 = -x1;
t15 = -x2;
t16 = -x3;
t29 = x7.*x8.*(1.77e+2./1.0e+3);
t30 = x7.*x9.*(1.77e+2./1.0e+3);
t31 = x7.*x10.*(1.77e+2./1.0e+3);
t32 = x8.*x9.*(1.77e+2./1.0e+3);
t33 = x8.*x10.*(1.77e+2./1.0e+3);
t34 = x9.*x10.*(1.77e+2./1.0e+3);
t17 = p_box1_1+t6;
t18 = p_box1_1+t7;
t19 = p_box2_1+t9;
t20 = p_box2_1+t10;
t21 = p_box3_1+t12;
t22 = p_box3_1+t13;
t35 = t2.*8.85e-2;
t36 = t3.*8.85e-2;
t37 = t4.*8.85e-2;
t38 = t5.*8.85e-2;
t39 = -t30;
t40 = -t31;
t41 = -t32;
t42 = -t33;
t49 = p_box2_1+t15+t31+t32;
t50 = t8+t31+t32+x2;
t51 = p_box3_1+t16+t29+t34;
t52 = t11+t29+t34+x3;
t23 = t17.^2;
t24 = t18.^2;
t25 = t19.^2;
t26 = t20.^2;
t27 = t21.^2;
t28 = t22.^2;
t43 = -t35;
t44 = -t36;
t45 = -t37;
t46 = -t38;
t55 = p_box1_1+t14+t32+t40;
t56 = p_box1_1+t14+t31+t41;
t57 = p_box3_1+t16+t33+t39;
t58 = p_box3_1+t16+t30+t42;
t59 = t19.*t49;
t60 = t19.*t50;
t61 = t20.*t49;
t62 = t20.*t50;
t63 = t21.*t51;
t64 = t21.*t52;
t65 = t22.*t51;
t66 = t22.*t52;
t47 = t23+t25+t27;
t48 = t24+t26+t28;
t67 = t17.*t55;
t68 = t17.*t56;
t69 = t18.*t55;
t70 = t18.*t56;
t71 = t21.*t57;
t72 = t21.*t58;
t73 = t22.*t57;
t74 = t22.*t58;
t75 = -t60;
t76 = -t62;
t77 = -t64;
t78 = -t66;
t79 = p_box1_1+t14+t37+t38+t43+t44;
t80 = p_box1_1+t14+t35+t36+t45+t46;
t81 = p_box2_1+t15+t36+t38+t43+t45;
t82 = p_box2_1+t15+t35+t37+t44+t46;
t53 = 1.0./t47;
t54 = 1.0./t48;
t83 = t17.*t79;
t84 = t17.*t80;
t85 = t18.*t79;
t86 = t18.*t80;
t87 = t19.*t81;
t88 = t19.*t82;
t89 = t20.*t81;
t90 = t20.*t82;
t91 = t63+t67+t88;
t92 = t59+t71+t84;
t93 = t65+t69+t90;
t94 = t61+t73+t86;
t95 = t68+t77+t87;
t96 = t72+t75+t83;
t97 = t70+t78+t89;
t98 = t74+t76+t85;
t99 = t53.*t91;
t100 = t53.*t92;
t101 = t54.*t93;
t102 = t54.*t94;
t103 = t53.*t95;
t104 = t53.*t96;
t105 = t54.*t97;
t106 = t54.*t98;
conx = [-t104;-t99;-t100;-t103;-t106;-t101;-t102;-t105;t104-1.0;t99-1.0;t100-1.0;t103-1.0;t106-1.0;t101-1.0;t102-1.0;t105-1.0];
