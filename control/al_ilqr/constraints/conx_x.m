function conx_x = conx_x(in1,in2)
%CONX_X
%    CONX_X = CONX_X(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    20-Aug-2021 19:54:09

p_box1_1 = in2(1);
p_box1_2 = in2(4);
p_box1_4 = in2(10);
p_box2_1 = in2(2);
p_box2_2 = in2(5);
p_box2_4 = in2(11);
p_box3_1 = in2(3);
p_box3_2 = in2(6);
p_box3_4 = in2(12);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
t2 = -p_box1_2;
t3 = -p_box1_4;
t4 = -p_box2_2;
t5 = -p_box2_4;
t6 = -p_box3_2;
t7 = -p_box3_4;
t8 = p_box1_1+t2;
t9 = p_box1_1+t3;
t10 = p_box2_1+t4;
t11 = p_box2_1+t5;
t12 = p_box3_1+t6;
t13 = p_box3_1+t7;
t14 = t8.^2;
t15 = t9.^2;
t16 = t10.^2;
t17 = t11.^2;
t18 = t12.^2;
t19 = t13.^2;
t20 = t8.*x7.*(3.0./2.5e+1);
t21 = t8.*x8.*(3.0./2.5e+1);
t22 = t9.*x7.*(3.0./2.5e+1);
t23 = t8.*x9.*(3.0./2.5e+1);
t24 = t9.*x8.*(3.0./2.5e+1);
t25 = t8.*x10.*(3.0./2.5e+1);
t26 = t9.*x9.*(3.0./2.5e+1);
t27 = t9.*x10.*(3.0./2.5e+1);
t28 = t10.*x7.*(3.0./2.5e+1);
t29 = t10.*x8.*(3.0./2.5e+1);
t30 = t11.*x7.*(3.0./2.5e+1);
t31 = t10.*x9.*(3.0./2.5e+1);
t32 = t11.*x8.*(3.0./2.5e+1);
t33 = t10.*x10.*(3.0./2.5e+1);
t34 = t11.*x9.*(3.0./2.5e+1);
t35 = t11.*x10.*(3.0./2.5e+1);
t36 = t12.*x7.*(3.0./2.5e+1);
t37 = t12.*x8.*(3.0./2.5e+1);
t38 = t13.*x7.*(3.0./2.5e+1);
t39 = t12.*x9.*(3.0./2.5e+1);
t40 = t13.*x8.*(3.0./2.5e+1);
t41 = t12.*x10.*(3.0./2.5e+1);
t42 = t13.*x9.*(3.0./2.5e+1);
t43 = t13.*x10.*(3.0./2.5e+1);
t44 = -t25;
t45 = -t27;
t46 = -t29;
t47 = -t32;
t48 = -t39;
t49 = -t42;
t50 = t14+t16+t18;
t51 = t15+t17+t19;
t66 = t21+t31+t41;
t67 = t24+t34+t43;
t52 = 1.0./t50;
t53 = 1.0./t51;
t68 = t23+t36+t46;
t69 = t28+t37+t44;
t70 = t20+t33+t48;
t71 = t26+t38+t47;
t72 = t30+t40+t45;
t73 = t22+t35+t49;
t54 = t8.*t52;
t55 = t9.*t53;
t56 = t10.*t52;
t57 = t11.*t53;
t58 = t12.*t52;
t59 = t13.*t53;
t74 = t52.*t66;
t75 = t53.*t67;
t76 = t52.*t68;
t77 = t52.*t69;
t78 = t52.*t70;
t79 = t53.*t71;
t80 = t53.*t72;
t81 = t53.*t73;
t60 = -t54;
t61 = -t55;
t62 = -t56;
t63 = -t57;
t64 = -t58;
t65 = -t59;
t82 = -t74;
t83 = -t75;
t84 = -t76;
t85 = -t77;
t86 = -t78;
t87 = -t79;
t88 = -t80;
t89 = -t81;
conx_x = reshape([t54,t54,t54,t54,t55,t55,t55,t55,t60,t60,t60,t60,t61,t61,t61,t61,t56,t56,t56,t56,t57,t57,t57,t57,t62,t62,t62,t62,t63,t63,t63,t63,t58,t58,t58,t58,t59,t59,t59,t59,t64,t64,t64,t64,t65,t65,t65,t65,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t78,t85,t86,t77,t81,t88,t89,t80,t86,t77,t78,t85,t89,t80,t81,t88,t74,t84,t82,t76,t75,t87,t83,t79,t82,t76,t74,t84,t83,t79,t75,t87,t84,t82,t76,t74,t87,t83,t79,t75,t76,t74,t84,t82,t79,t75,t87,t83,t77,t78,t85,t86,t80,t81,t88,t89,t85,t86,t77,t78,t88,t89,t80,t81],[16,10]);
