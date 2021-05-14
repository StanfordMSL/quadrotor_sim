function con_u = motor_con_u(in1,in2,in3)
%MOTOR_CON_U
%    CON_U = MOTOR_CON_U(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    13-May-2021 22:47:49

u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
u_p2 = in3(2,:);
u_p3 = in3(3,:);
u_p4 = in3(4,:);
t2 = u1.*2.82556115644567e+7;
t3 = u2.*4.916795684662522e+7;
t4 = u3.*4.916795684662522e+7;
t5 = u_p2.*4.916795684662522e+7;
t6 = u_p3.*4.916795684662522e+7;
t7 = u4.*7.092158502678632e+7;
t8 = u_p4.*7.092158502678632e+7;
t15 = u2.*3.096942736443277e+5;
t16 = u3.*3.096942736443277e+5;
t17 = u4.*3.096942736443277e+5;
t22 = u2.*u4.*(-3.096942736443277e+5);
t23 = u3.*u4.*(-3.096942736443277e+5);
t9 = -t3;
t10 = -t4;
t11 = -t5;
t12 = -t6;
t13 = -t7;
t14 = -t8;
t18 = -t15;
t19 = -t16;
t20 = t15.*u4;
t21 = t16.*u4;
t24 = t17+4.916795684662522e+7;
t25 = t17-4.916795684662522e+7;
t26 = t15+t16+7.092158502678632e+7;
t29 = t15+t16-7.092158502678632e+7;
t27 = t16+t18+7.092158502678632e+7;
t28 = t15+t19+7.092158502678632e+7;
t30 = t2+t3+t6+t7+t10+t11+t14+t20+t21;
t31 = t2+t5+t6+t8+t9+t10+t13+t20+t23;
t32 = t2+t3+t4+t8+t11+t12+t13+t21+t22;
t33 = t2+t4+t5+t7+t9+t12+t14+t22+t23;
t34 = 1.0./sqrt(t30);
t35 = 1.0./sqrt(t31);
t36 = 1.0./sqrt(t32);
t37 = 1.0./sqrt(t33);
t38 = t34.*1.412780578222835e+7;
t39 = t35.*1.412780578222835e+7;
t40 = t36.*1.412780578222835e+7;
t41 = t37.*1.412780578222835e+7;
t42 = (t24.*t34)./2.0;
t43 = (t25.*t34)./2.0;
t44 = (t24.*t35)./2.0;
t45 = (t24.*t36)./2.0;
t46 = (t25.*t35)./2.0;
t47 = (t25.*t36)./2.0;
t48 = (t24.*t37)./2.0;
t49 = (t25.*t37)./2.0;
t50 = (t26.*t34)./2.0;
t51 = (t27.*t35)./2.0;
t52 = (t28.*t36)./2.0;
t53 = (t29.*t37)./2.0;
con_u = reshape([t39,t40,t38,t41,-t39,-t40,-t38,-t41,t46,-t47,t42,-t48,-t46,t47,-t42,t48,-t44,t45,t43,-t49,t44,-t45,-t43,t49,-t51,-t52,t50,-t53,t51,t52,-t50,t53],[8,4]);
