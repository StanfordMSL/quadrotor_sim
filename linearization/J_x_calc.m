function J_x = J_x_calc(q_w,q_x,q_y,q_z,u1,u2,u3,u4,v_x,v_y,v_z,w_x,w_y,w_z)
%J_X_CALC
%    J_X = J_X_CALC(Q_W,Q_X,Q_Y,Q_Z,U1,U2,U3,U4,V_X,V_Y,V_Z,W_X,W_Y,W_Z)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    23-Feb-2021 14:26:17

t2 = q_w.^2;
t3 = q_x.^2;
t4 = q_y.^2;
t5 = q_z.^2;
t6 = u1.^2;
t7 = u2.^2;
t8 = u3.^2;
t9 = u4.^2;
t10 = q_w./2.0;
t11 = q_x./2.0;
t12 = q_y./2.0;
t13 = q_z./2.0;
t14 = w_x./2.0;
t15 = w_y./2.0;
t16 = w_z./2.0;
t23 = q_w.*v_z.*(4.0./1.3e+1);
t24 = q_x.*v_z.*(4.0./1.3e+1);
t25 = q_y.*v_z.*(4.0./1.3e+1);
t26 = q_z.*v_z.*(4.0./1.3e+1);
t27 = q_w.*q_z.*(2.0e+1./1.3e+1);
t28 = q_x.*q_y.*(2.0e+1./1.3e+1);
t29 = (q_w.*q_z)./7.7e+1;
t30 = (q_x.*q_y)./7.7e+1;
t31 = q_w.*v_x.*(2.0e+1./1.3e+1);
t32 = (q_w.*v_x)./7.7e+1;
t33 = q_w.*v_y.*(2.0e+1./1.3e+1);
t34 = q_x.*v_x.*(2.0e+1./1.3e+1);
t35 = (q_w.*v_y)./7.7e+1;
t36 = (q_x.*v_x)./7.7e+1;
t38 = q_x.*v_y.*(2.0e+1./1.3e+1);
t39 = q_y.*v_x.*(2.0e+1./1.3e+1);
t40 = (q_w.*v_z)./7.7e+1;
t41 = (q_x.*v_y)./7.7e+1;
t42 = (q_y.*v_x)./7.7e+1;
t43 = q_y.*v_y.*(2.0e+1./1.3e+1);
t44 = q_z.*v_x.*(2.0e+1./1.3e+1);
t45 = (q_x.*v_z)./7.7e+1;
t46 = (q_y.*v_y)./7.7e+1;
t47 = (q_z.*v_x)./7.7e+1;
t49 = q_z.*v_y.*(2.0e+1./1.3e+1);
t50 = (q_y.*v_z)./7.7e+1;
t51 = (q_z.*v_y)./7.7e+1;
t53 = (q_z.*v_z)./7.7e+1;
t75 = w_z.*(9.7e+1./1.54e+2);
t17 = -t11;
t18 = -t12;
t19 = -t13;
t20 = -t14;
t21 = -t15;
t22 = -t16;
t37 = -t23;
t48 = -t25;
t52 = -t26;
t54 = t2.*(1.0e+1./1.3e+1);
t55 = t2./1.54e+2;
t56 = t3.*(1.0e+1./1.3e+1);
t57 = t3./1.54e+2;
t58 = t4.*(1.0e+1./1.3e+1);
t59 = t4./1.54e+2;
t60 = t5.*(1.0e+1./1.3e+1);
t61 = t5./1.54e+2;
t62 = -t28;
t63 = -t30;
t64 = -t31;
t65 = -t33;
t66 = -t34;
t67 = -t35;
t68 = -t36;
t69 = -t38;
t70 = -t43;
t71 = -t44;
t72 = -t45;
t73 = -t46;
t74 = -t53;
t78 = t6+t7+t8+t9;
t76 = -t54;
t77 = -t55;
t79 = t47+t67+t72;
t80 = t68+t73+t74;
t81 = q_w.*t78.*2.7224e-8;
t82 = q_x.*t78.*2.7224e-8;
t83 = q_y.*t78.*2.7224e-8;
t84 = q_z.*t78.*2.7224e-8;
t85 = -t82;
t86 = t37+t39+t69+t81;
t87 = t48+t49+t64+t83;
t89 = t52+t66+t70+t84;
t88 = t24+t65+t71+t85;
J_x = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,-t56+t58+t60+t76,-t27+t62,q_w.*q_y.*(2.0e+1./1.3e+1)-q_x.*q_z.*(2.0e+1./1.3e+1),0.0,0.0,0.0,0.0,-t57+t59+t61+t77,t29+t63,q_w.*q_y.*(-2.0./2.51e+2)-q_x.*q_z.*(2.0./2.51e+2),0.0,1.0,0.0,t27+t62,t56-t58+t60+t76,q_w.*q_x.*(-2.0e+1./1.3e+1)-q_y.*q_z.*(2.0e+1./1.3e+1),0.0,0.0,0.0,0.0,-t29+t63,t57-t59+t61+t77,q_w.*q_x.*(2.0./2.51e+2)-q_y.*q_z.*(2.0./2.51e+2),0.0,0.0,1.0,q_w.*q_y.*(-4.0./1.3e+1)-q_x.*q_z.*(4.0./1.3e+1),q_w.*q_x.*(4.0./1.3e+1)-q_y.*q_z.*(4.0./1.3e+1),t2.*(-2.0./1.3e+1)+t3.*(2.0./1.3e+1)+t4.*(2.0./1.3e+1)-t5.*(2.0./1.3e+1),0.0,0.0,0.0,0.0,(q_w.*q_y)./7.7e+1-(q_x.*q_z)./7.7e+1,q_w.*q_x.*(-1.0./7.7e+1)-(q_y.*q_z)./7.7e+1,t2.*(-1.0./2.51e+2)+t3./2.51e+2+t4./2.51e+2-t5./2.51e+2,0.0,0.0,0.0,t87,t88,t86,0.0,t14,t15,t16,-t32+t50-t51,t79,q_w.*v_z.*(-2.0./2.51e+2)+q_x.*v_y.*(2.0./2.51e+2)-q_y.*v_x.*(2.0./2.51e+2),0.0,0.0,0.0,t89,t23+t38-t39-t81,t88,t20,0.0,t22,t15,t80,-t40+t41-t42,q_w.*v_y.*(2.0./2.51e+2)+q_x.*v_z.*(2.0./2.51e+2)-q_z.*v_x.*(2.0./2.51e+2),0.0,0.0,0.0,t86,t89,t25+t31-t49-t83,t21,t16,0.0,t20,t40-t41+t42,t80,q_w.*v_x.*(-2.0./2.51e+2)+q_y.*v_z.*(2.0./2.51e+2)-q_z.*v_y.*(2.0./2.51e+2),0.0,0.0,0.0,-t24+t33+t44+t82,t87,t89,t22,t21,t14,0.0,t79,t32-t50+t51,q_x.*v_x.*(-2.0./2.51e+2)-q_y.*v_y.*(2.0./2.51e+2)-q_z.*v_z.*(2.0./2.51e+2),0.0,0.0,0.0,0.0,0.0,0.0,t17,t10,t13,t18,-1.0./1.54e+2,t75,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t18,t19,t10,t11,-t75,-1.0./1.54e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t19,t12,t17,t10,w_y.*(-9.7e+1./1.54e+2),w_x.*(9.7e+1./1.54e+2),-1.0./2.51e+2],[13,13]);