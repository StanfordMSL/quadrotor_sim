function x_upd = quadcopter_act(in1,in2,in3,in4)
%QUADCOPTER_ACT
%    X_UPD = QUADCOPTER_ACT(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Aug-2021 13:03:14

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
t2 = sign(w_m1);
t3 = sign(w_m2);
t4 = sign(w_m3);
t5 = sign(w_m4);
t6 = q1.^2;
t7 = q2.^2;
t8 = q3.^2;
t9 = q4.^2;
t10 = w_m1.^2;
t11 = w_m2.^2;
t12 = w_m3.^2;
t13 = w_m4.^2;
t14 = -q1;
t23 = q1.*q2.*(8.0./5.0);
t24 = q1.*q3.*(8.0./5.0);
t25 = q1.*q4.*(8.0./5.0);
t26 = q2.*q3.*(8.0./5.0);
t27 = q2.*q4.*(8.0./5.0);
t28 = q3.*q4.*(8.0./5.0);
t32 = (q1.*w1)./2.0e+3;
t33 = (q1.*w2)./2.0e+3;
t34 = (q2.*w1)./2.0e+3;
t35 = (q1.*w3)./2.0e+3;
t36 = (q2.*w2)./2.0e+3;
t37 = (q3.*w1)./2.0e+3;
t38 = (q2.*w3)./2.0e+3;
t39 = (q3.*w2)./2.0e+3;
t40 = (q4.*w1)./2.0e+3;
t41 = (q3.*w3)./2.0e+3;
t42 = (q4.*w2)./2.0e+3;
t43 = (q4.*w3)./2.0e+3;
t15 = t2.*t10;
t16 = t3.*t11;
t17 = t4.*t12;
t18 = t5.*t13;
t19 = t6.*(4.0./5.0);
t20 = t7.*(4.0./5.0);
t21 = t8.*(4.0./5.0);
t22 = t9.*(4.0./5.0);
t44 = -t37;
t45 = -t38;
t46 = -t42;
t54 = t14+t34+t39+t43;
t29 = -t20;
t30 = -t21;
t31 = -t22;
t47 = t15+t16+t17+t18;
t48 = q2+t32+t41+t46;
t49 = q3+t33+t40+t45;
t50 = q4+t35+t36+t44;
t55 = abs(t54);
t51 = abs(t48);
t52 = abs(t49);
t53 = abs(t50);
t59 = t55.^2;
t56 = t51.^2;
t57 = t52.^2;
t58 = t53.^2;
t60 = t56+t57+t58+t59;
t61 = 1.0./sqrt(t60);
et1 = t15.*(-1.386e-8)+t16.*1.386e-8+t17.*1.386e-8-t18.*1.386e-8+tau_ext1+w1.*9.99999999e-1+wt11;
et2 = t6.*v1.*(-1.0e-9)-(t7.*v1)./1.0e+9+(t8.*v1)./1.0e+9+(t9.*v1)./1.0e+9-(w2.*w3)./2.5e+3+w2.*w_m1.*7.799999999999999e-7+w2.*w_m2.*7.799999999999999e-7-w2.*w_m3.*7.799999999999999e-7;
et3 = w2.*w_m4.*(-7.799999999999999e-7)+(q1.*q3.*v3)./5.0e+8-(q1.*q4.*v2)./5.0e+8-(q2.*q3.*v2)./5.0e+8-(q2.*q4.*v3)./5.0e+8;
et4 = t15.*(-8.6625e-9)+t16.*8.6625e-9-t17.*8.6625e-9+t18.*8.6625e-9+tau_ext2.*(5.0./8.0)+w2.*9.99999999375e-1+wt12;
et5 = t6.*v2.*(-6.25e-10)+(t7.*v2)./1.6e+9-(t8.*v2)./1.6e+9+(t9.*v2)./1.6e+9+(w1.*w3)./1.6e+3-w1.*w_m1.*4.875e-7-w1.*w_m2.*4.875e-7;
et6 = w1.*w_m3.*4.875e-7+w1.*w_m4.*4.875e-7-(q1.*q2.*v3)./8.0e+8+(q1.*q4.*v1)./8.0e+8-(q2.*q3.*v1)./8.0e+8-(q3.*q4.*v3)./8.0e+8;
et7 = t15.*(-1.5015e-9)-t16.*1.5015e-9+t17.*1.5015e-9+t18.*1.5015e-9+tau_ext3./2.0;
et8 = w3.*9.999999995e-1+wt13-(t6.*v3)./2.0e+9+(t7.*v3)./2.0e+9+(t8.*v3)./2.0e+9-(t9.*v3)./2.0e+9-w1.*w2.*3.0e-4+(q1.*q2.*v2)./1.0e+9-(q1.*q3.*v1)./1.0e+9-(q2.*q4.*v1)./1.0e+9-(q3.*q4.*v2)./1.0e+9;
mt1 = [p1+v1./1.0e+3+wt1,p2+v2./1.0e+3+wt2,p3+v3./1.0e+3+wt3,F_ext1./5.3e+2+v1+wt4-(v1.*(t19+t20+t30+t31))./5.3e+2-(v3.*(t24+t27))./5.3e+2+(v2.*(t25-t26))./5.3e+2+t47.*(q1.*q3+q2.*q4).*8.716981132075471e-10];
mt2 = [F_ext2./5.3e+2+v2+wt5-(v2.*(t19+t21+t29+t31))./5.3e+2-(v1.*(t25+t26))./5.3e+2+(v3.*(t23-t28))./5.3e+2-t47.*(q1.*q2-q3.*q4).*8.716981132075471e-10];
mt3 = [F_ext3./5.3e+2+v3+wt6-(v3.*(t19+t22+t29+t30))./5.3e+2-(v2.*(t23+t28))./5.3e+2+(v1.*(t24-t27))./5.3e+2+t47.*(t6-t7-t8+t9).*4.358490566037736e-10-9.81e-3,wt7-t54.*t61,wt8+t48.*t61,wt9+t49.*t61,wt10+t50.*t61,et1+et2+et3,et4+et5+et6,et7+et8];
x_upd = reshape([mt1,mt2,mt3],13,1);
