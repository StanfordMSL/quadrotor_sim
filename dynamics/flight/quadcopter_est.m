function x_upd = quadcopter_est(in1,in2,in3,in4)
%QUADCOPTER_EST
%    X_UPD = QUADCOPTER_EST(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    09-Jul-2021 21:21:31

F_ext1 = in3(1,:);
F_ext2 = in3(2,:);
F_ext3 = in3(3,:);
p1 = in1(1,:);
p2 = in1(2,:);
p3 = in1(3,:);
q1 = in1(7,:);
q2 = in1(8,:);
q3 = in1(9,:);
q4 = in1(10,:);
tau_ext1 = in3(4,:);
tau_ext2 = in3(5,:);
tau_ext3 = in3(6,:);
v1 = in1(4,:);
v2 = in1(5,:);
v3 = in1(6,:);
w1 = in1(11,:);
w2 = in1(12,:);
w3 = in1(13,:);
w_m1 = in2(1,:);
w_m2 = in2(2,:);
w_m3 = in2(3,:);
w_m4 = in2(4,:);
wt1 = in4(1,:);
wt2 = in4(2,:);
wt3 = in4(3,:);
wt4 = in4(4,:);
wt5 = in4(5,:);
wt6 = in4(6,:);
wt7 = in4(7,:);
wt8 = in4(8,:);
wt9 = in4(9,:);
wt10 = in4(10,:);
wt11 = in4(11,:);
wt12 = in4(12,:);
wt13 = in4(13,:);
t2 = q1.^2;
t3 = q2.^2;
t4 = q3.^2;
t5 = q4.^2;
t6 = w_m1.^2;
t7 = w_m2.^2;
t8 = w_m3.^2;
t9 = w_m4.^2;
t10 = q1.*q2.*2.0;
t11 = q1.*q3.*2.0;
t12 = q1.*q4.*2.0;
t13 = q2.*q3.*2.0;
t14 = q2.*q4.*2.0;
t15 = q3.*q4.*2.0;
t16 = -q1;
t22 = (q1.*w1)./2.0e+2;
t23 = (q1.*w2)./2.0e+2;
t24 = (q2.*w1)./2.0e+2;
t25 = (q1.*w3)./2.0e+2;
t26 = (q2.*w2)./2.0e+2;
t27 = (q3.*w1)./2.0e+2;
t28 = (q2.*w3)./2.0e+2;
t29 = (q3.*w2)./2.0e+2;
t30 = (q4.*w1)./2.0e+2;
t31 = (q3.*w3)./2.0e+2;
t32 = (q4.*w2)./2.0e+2;
t33 = (q4.*w3)./2.0e+2;
t37 = (q1.*q2)./5.0e+8;
t38 = (q1.*q3)./5.0e+8;
t39 = (q1.*q4)./5.0e+8;
t40 = (q2.*q3)./5.0e+8;
t41 = (q2.*q4)./5.0e+8;
t42 = (q3.*q4)./5.0e+8;
t17 = -t11;
t18 = -t13;
t19 = -t2;
t20 = -t3;
t21 = -t5;
t34 = -t27;
t35 = -t28;
t36 = -t32;
t43 = t2./1.0e+9;
t44 = t3./1.0e+9;
t45 = t4./1.0e+9;
t46 = t5./1.0e+9;
t60 = t16+t24+t29+t33;
t69 = t6.*5.84e-6;
t70 = t7.*5.84e-6;
t71 = t8.*5.84e-6;
t72 = t9.*5.84e-6;
t73 = t6.*1.776082474226804e-7;
t74 = t7.*1.776082474226804e-7;
t75 = t8.*1.776082474226804e-7;
t76 = t9.*1.776082474226804e-7;
t47 = -t44;
t48 = -t45;
t49 = -t46;
t50 = t10+t14+t15+t17;
t52 = q2+t22+t31+t36;
t53 = q3+t23+t30+t35;
t54 = q4+t25+t26+t34;
t55 = t2+t4+t12+t13+t20+t21;
t59 = t4+t5+t12+t18+t19+t20;
t62 = abs(t60);
t66 = -v1.*(t2+t3-t4-t12+t13+t21);
t67 = v1.*(t2+t3-t4-t12+t13+t21);
t77 = -t73;
t51 = t50.*v3;
t56 = abs(t52);
t57 = abs(t53);
t58 = abs(t54);
t61 = t55.*v2;
t68 = t62.^2;
t63 = t56.^2;
t64 = t57.^2;
t65 = t58.^2;
t78 = t51+t61+t67;
t79 = t78.^2;
t80 = t63+t64+t65+t68;
t81 = t79.*1.5e-9;
t82 = 1.0./sqrt(t80);
t83 = t69+t70+t71+t72+t81;
x_upd = [p1+v1./1.0e+2+wt1;p2+v2./1.0e+2+wt2;p3+v3./1.0e+2+wt3;F_ext1./1.5e+2+v1+wt4-(v1.*(t43+t44+t48+t49))./1.5e+2-(v3.*(t38+t41))./1.5e+2+(v2.*(t39-t40))./1.5e+2+(q1.*q3.*t83)./7.5e+1+(q2.*q4.*t83)./7.5e+1;F_ext2./1.5e+2+v2+wt5-(v2.*(t43+t45+t47+t49))./1.5e+2-(v1.*(t39+t40))./1.5e+2+(v3.*(t37-t42))./1.5e+2-(q1.*q2.*t83)./7.5e+1+(q3.*q4.*t83)./7.5e+1;F_ext3./1.5e+2+v3+wt6-(v3.*(t43+t46+t47+t48))./1.5e+2-(v2.*(t37+t42))./1.5e+2+(t2.*t83)./1.5e+2-(t3.*t83)./1.5e+2-(t4.*t83)./1.5e+2+(t5.*t83)./1.5e+2+(v1.*(t38-t41))./1.5e+2-9.81e-2;wt7-t60.*t82;wt8+t52.*t82;wt9+t53.*t82;wt10+t54.*t82;t74+t75-t76+t77+tau_ext1.*(1.0e+2./2.91e+2)+w1.*9.999999996563574e-1+wt11-(t2.*v1)./2.91e+9-(t3.*v1)./2.91e+9+(t4.*v1)./2.91e+9+(t5.*v1)./2.91e+9-w2.*w3.*8.969072164948454e-3+(q1.*q3.*v3)./1.455e+9-(q1.*q4.*v2)./1.455e+9-(q2.*q3.*v2)./1.455e+9-(q2.*q4.*v3)./1.455e+9;t74-t75+t76+t77+tau_ext2.*(1.0e+2./2.91e+2)+w2.*9.999999996563574e-1+wt12-(t2.*v2)./2.91e+9+(t3.*v2)./2.91e+9-(t4.*v2)./2.91e+9+(t5.*v2)./2.91e+9+w1.*w3.*8.969072164948454e-3-(q1.*q2.*v3)./1.455e+9+(q1.*q4.*v1)./1.455e+9-(q2.*q3.*v1)./1.455e+9-(q3.*q4.*v3)./1.455e+9;t6.*(-6.347826086956522e-8)-t7.*6.347826086956522e-8+t8.*6.347826086956522e-8+t9.*6.347826086956522e-8+tau_ext3.*(2.5e+1./1.38e+2)+w3.*9.999999998188406e-1+wt13-(t2.*v3)./5.52e+9+(t3.*v3)./5.52e+9+(t4.*v3)./5.52e+9-(t5.*v3)./5.52e+9+(q1.*q2.*v2)./2.76e+9-(q1.*q3.*v1)./2.76e+9-(q2.*q4.*v1)./2.76e+9-(q3.*q4.*v2)./2.76e+9];
