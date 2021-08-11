function x_upd = quadcopter_est(in1,in2,in3,in4)
%QUADCOPTER_EST
%    X_UPD = QUADCOPTER_EST(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    10-Aug-2021 21:37:28

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
t6 = q1.*q2;
t7 = q1.*q3;
t8 = q2.*q4;
t9 = q3.*q4;
t10 = q1.^2;
t11 = q2.^2;
t12 = q3.^2;
t13 = q4.^2;
t14 = w_m1.^2;
t15 = w_m2.^2;
t16 = w_m3.^2;
t17 = w_m4.^2;
t18 = -q1;
t24 = (q1.*q4)./2.0;
t25 = (q2.*q3)./2.0;
t32 = (q1.*w1)./4.0e+2;
t33 = (q1.*w2)./4.0e+2;
t34 = (q2.*w1)./4.0e+2;
t35 = (q1.*w3)./4.0e+2;
t36 = (q2.*w2)./4.0e+2;
t37 = (q3.*w1)./4.0e+2;
t38 = (q2.*w3)./4.0e+2;
t39 = (q3.*w2)./4.0e+2;
t40 = (q4.*w1)./4.0e+2;
t41 = (q3.*w3)./4.0e+2;
t42 = (q4.*w2)./4.0e+2;
t43 = (q4.*w3)./4.0e+2;
t19 = -t9;
t20 = t2.*t14;
t21 = t3.*t15;
t22 = t4.*t16;
t23 = t5.*t17;
t26 = t10./4.0;
t27 = t11./4.0;
t28 = t12./4.0;
t29 = t13./4.0;
t30 = t7+t8;
t44 = -t37;
t45 = -t38;
t46 = -t42;
t55 = t18+t34+t39+t43;
t31 = -t29;
t47 = t6+t19;
t48 = t20+t21+t22+t23;
t49 = q2+t32+t41+t46;
t50 = q3+t33+t40+t45;
t51 = q4+t35+t36+t44;
t56 = abs(t55);
t52 = abs(t49);
t53 = abs(t50);
t54 = abs(t51);
t60 = t56.^2;
t57 = t52.^2;
t58 = t53.^2;
t59 = t54.^2;
t61 = t57+t58+t59+t60;
t62 = 1.0./sqrt(t61);
x_upd = [p1+v1./2.0e+2+wt1;p2+v2./2.0e+2+wt2;p3+v3./2.0e+2+wt3;F_ext1./1.06e+2+v1+wt4+t30.*t48.*4.433962264150943e-9-(t30.*v3)./1.06e+2-(v1.*(t26+t27-t28+t31))./1.06e+2+(v2.*(t24-t25))./1.06e+2;F_ext2./1.06e+2+v2+wt5-(v1.*(t24+t25))./1.06e+2-t47.*t48.*4.433962264150943e-9+(t47.*v3)./1.06e+2-(v2.*(t26-t27+t28+t31))./1.06e+2;F_ext3./1.06e+2+v3+wt6-(v3.*(t10./2.0-t11./2.0-t12./2.0+t13./2.0))./1.06e+2+t48.*(t10-t11-t12+t13).*2.216981132075472e-9+(v1.*(t7./2.0-t8./2.0))./1.06e+2-(v2.*(t6./2.0+t9./2.0))./1.06e+2-4.905e-2;wt7-t55.*t62;wt8+t49.*t62;wt9+t50.*t62;wt10+t51.*t62;t20.*(-7.05e-8)+t21.*7.05e-8+t22.*7.05e-8-t23.*7.05e-8+tau_ext1.*5.0+w1.*9.9995e-1+wt11+(t7.*v3)./1.0e+4-(t8.*v3)./1.0e+4-(t10.*v1)./2.0e+4-(t11.*v1)./2.0e+4+(t12.*v1)./2.0e+4+(t13.*v1)./2.0e+4-(w2.*w3)./5.0e+2-(q1.*q4.*v2)./1.0e+4-(q2.*q3.*v2)./1.0e+4;t20.*(-4.40625e-8)+t21.*4.40625e-8-t22.*4.40625e-8+t23.*4.40625e-8+tau_ext2.*(2.5e+1./8.0)+w2.*9.9996875e-1+wt12-(t6.*v3)./1.6e+4-(t9.*v3)./1.6e+4-(t10.*v2)./3.2e+4+(t11.*v2)./3.2e+4-(t12.*v2)./3.2e+4+(t13.*v2)./3.2e+4+(w1.*w3)./3.2e+2+(q1.*q4.*v1)./1.6e+4-(q2.*q3.*v1)./1.6e+4;t20.*(-7.6375e-9)-t21.*7.6375e-9+t22.*7.6375e-9+t23.*7.6375e-9+tau_ext3.*(5.0./2.0)+w3.*9.99975e-1+wt13+(t6.*v2)./2.0e+4-(t7.*v1)./2.0e+4-(t8.*v1)./2.0e+4-(t9.*v2)./2.0e+4-(t10.*v3)./4.0e+4+(t11.*v3)./4.0e+4+(t12.*v3)./4.0e+4-(t13.*v3)./4.0e+4-w1.*w2.*1.5e-3];
