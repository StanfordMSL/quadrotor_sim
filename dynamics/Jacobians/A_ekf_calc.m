function A_ekf = A_ekf_calc(in1,in2)
%A_EKF_CALC
%    A_EKF = A_EKF_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-Aug-2021 18:53:51

u_ses1 = in2(1,:);
u_ses2 = in2(2,:);
u_ses3 = in2(3,:);
u_ses4 = in2(4,:);
x_ses4 = in1(4,:);
x_ses5 = in1(5,:);
x_ses6 = in1(6,:);
x_ses7 = in1(7,:);
x_ses8 = in1(8,:);
x_ses9 = in1(9,:);
x_ses10 = in1(10,:);
x_ses11 = in1(11,:);
x_ses12 = in1(12,:);
x_ses13 = in1(13,:);
t2 = sign(u_ses1);
t3 = sign(u_ses2);
t4 = sign(u_ses3);
t5 = sign(u_ses4);
t6 = x_ses7.*x_ses11;
t7 = x_ses7.*x_ses12;
t8 = x_ses8.*x_ses11;
t9 = x_ses7.*x_ses13;
t10 = x_ses8.*x_ses12;
t11 = x_ses9.*x_ses11;
t12 = x_ses8.*x_ses13;
t13 = x_ses9.*x_ses12;
t14 = x_ses10.*x_ses11;
t15 = x_ses9.*x_ses13;
t16 = x_ses10.*x_ses12;
t17 = x_ses10.*x_ses13;
t18 = u_ses1.^2;
t19 = u_ses2.^2;
t20 = u_ses3.^2;
t21 = u_ses4.^2;
t22 = x_ses7.^2;
t23 = x_ses8.^2;
t24 = x_ses9.^2;
t25 = x_ses10.^2;
t26 = x_ses11.^2;
t27 = x_ses12.^2;
t28 = x_ses13.^2;
t32 = x_ses7.*4.0e+2;
t33 = x_ses8.*4.0e+2;
t34 = x_ses9.*4.0e+2;
t35 = x_ses10.*4.0e+2;
t29 = -t11;
t30 = -t12;
t31 = -t16;
t36 = t2.*t18;
t37 = t3.*t19;
t38 = t4.*t20;
t39 = t5.*t21;
t40 = -t32;
t41 = t22+t23+t24+t25;
t43 = t26+t27+t28+1.6e+5;
t42 = t8+t13+t17+t40;
t44 = t6+t15+t31+t33;
t45 = t7+t14+t30+t34;
t46 = t9+t10+t29+t35;
t47 = 1.0./sqrt(t41);
t49 = 1.0./sqrt(t43);
t51 = t36+t37+t38+t39;
t48 = t47.^3;
t50 = t49.^3;
t52 = t47.*t49.*x_ses7;
t53 = t47.*t49.*x_ses8;
t54 = t47.*t49.*x_ses9;
t55 = t47.*t49.*x_ses10;
t56 = t47.*t49.*x_ses11;
t57 = t47.*t49.*x_ses12;
t58 = t47.*t49.*x_ses13;
t65 = t47.*t49.*4.0e+2;
t66 = t51.*x_ses7.*3.867924528301887e-9;
t67 = t51.*x_ses8.*3.867924528301887e-9;
t68 = t51.*x_ses9.*3.867924528301887e-9;
t69 = t51.*x_ses10.*3.867924528301887e-9;
t59 = -t53;
t60 = -t54;
t61 = -t55;
t62 = -t56;
t63 = -t57;
t64 = -t58;
t70 = -t67;
A_ekf = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0e+2,0.0,0.0,5.29e+2./5.3e+2,0.0,0.0,0.0,0.0,0.0,0.0,t22.*(-5.0e-9)-t23./2.0e+8+t24./2.0e+8+t25./2.0e+8,(x_ses7.*x_ses10)./1.6e+8-(x_ses8.*x_ses9)./1.6e+8,x_ses7.*x_ses9.*(-5.0e-9)-(x_ses8.*x_ses10)./2.0e+8,0.0,1.0./2.0e+2,0.0,0.0,5.29e+2./5.3e+2,0.0,0.0,0.0,0.0,0.0,x_ses7.*x_ses10.*(-1.0e-8)-(x_ses8.*x_ses9)./1.0e+8,t22.*(-3.125e-9)+t23./3.2e+8-t24./3.2e+8+t25./3.2e+8,(x_ses7.*x_ses8)./2.0e+8-(x_ses9.*x_ses10)./2.0e+8,0.0,0.0,1.0./2.0e+2,0.0,0.0,9.990566037735849e-1,0.0,0.0,0.0,0.0,(x_ses7.*x_ses9)./1.0e+8-(x_ses8.*x_ses10)./1.0e+8,x_ses7.*x_ses8.*(-6.25e-9)-(x_ses9.*x_ses10)./1.6e+8,t22.*(-2.5e-9)+t23./4.0e+8+t24./4.0e+8-t25./4.0e+8,0.0,0.0,0.0,t68,t70,t66,t65+t42.*t48.*t49.*x_ses7,t56-t44.*t48.*t49.*x_ses7,t57-t45.*t48.*t49.*x_ses7,t58-t46.*t48.*t49.*x_ses7,x_ses4.*x_ses7.*(-1.0e-8)-(x_ses5.*x_ses10)./1.0e+8+(x_ses6.*x_ses9)./1.0e+8,x_ses5.*x_ses7.*(-6.25e-9)+(x_ses4.*x_ses10)./1.6e+8-(x_ses6.*x_ses8)./1.6e+8,x_ses4.*x_ses9.*(-5.0e-9)+(x_ses5.*x_ses8)./2.0e+8-(x_ses6.*x_ses7)./2.0e+8,0.0,0.0,0.0,t69,-t66,t70,t62+t42.*t48.*t49.*x_ses8,t65-t44.*t48.*t49.*x_ses8,t64-t45.*t48.*t49.*x_ses8,t57-t46.*t48.*t49.*x_ses8,x_ses4.*x_ses8.*(-1.0e-8)-(x_ses5.*x_ses9)./1.0e+8-(x_ses6.*x_ses10)./1.0e+8,x_ses4.*x_ses9.*(-6.25e-9)+(x_ses5.*x_ses8)./1.6e+8-(x_ses6.*x_ses7)./1.6e+8,(x_ses5.*x_ses7)./2.0e+8-(x_ses4.*x_ses10)./2.0e+8+(x_ses6.*x_ses8)./2.0e+8,0.0,0.0,0.0,t66,t69,-t68,t63+t42.*t48.*t49.*x_ses9,t58-t44.*t48.*t49.*x_ses9,t65-t45.*t48.*t49.*x_ses9,t62-t46.*t48.*t49.*x_ses9,(x_ses4.*x_ses9)./1.0e+8-(x_ses5.*x_ses8)./1.0e+8+(x_ses6.*x_ses7)./1.0e+8,x_ses4.*x_ses8.*(-6.25e-9)-(x_ses5.*x_ses9)./1.6e+8-(x_ses6.*x_ses10)./1.6e+8,x_ses4.*x_ses7.*(-5.0e-9)-(x_ses5.*x_ses10)./2.0e+8+(x_ses6.*x_ses9)./2.0e+8,0.0,0.0,0.0,t67,t68,t69,t64+t42.*t48.*t49.*x_ses10,t63-t44.*t48.*t49.*x_ses10,t56-t45.*t48.*t49.*x_ses10,t65-t46.*t48.*t49.*x_ses10,x_ses5.*x_ses7.*(-1.0e-8)+(x_ses4.*x_ses10)./1.0e+8-(x_ses6.*x_ses8)./1.0e+8,(x_ses4.*x_ses7)./1.6e+8+(x_ses5.*x_ses10)./1.6e+8-(x_ses6.*x_ses9)./1.6e+8,x_ses4.*x_ses8.*(-5.0e-9)-(x_ses5.*x_ses9)./2.0e+8-(x_ses6.*x_ses10)./2.0e+8,0.0,0.0,0.0,0.0,0.0,0.0,t59+t42.*t47.*t50.*x_ses11,t52-t44.*t47.*t50.*x_ses11,t55-t45.*t47.*t50.*x_ses11,t60-t46.*t47.*t50.*x_ses11,9.99999995e-1,u_ses1.*(-2.455729166666667e-6)-u_ses2.*2.455729166666667e-6+u_ses3.*2.455729166666667e-6+u_ses4.*2.455729166666667e-6+x_ses13./3.2e+2,x_ses12.*(-1.5e-3),0.0,0.0,0.0,0.0,0.0,0.0,t60+t42.*t47.*t50.*x_ses12,t61-t44.*t47.*t50.*x_ses12,t52-t45.*t47.*t50.*x_ses12,t53-t46.*t47.*t50.*x_ses12,u_ses1.*3.929166666666667e-6+u_ses2.*3.929166666666667e-6-u_ses3.*3.929166666666667e-6-u_ses4.*3.929166666666667e-6-x_ses13./5.0e+2,9.99999996875e-1,x_ses11.*(-1.5e-3),0.0,0.0,0.0,0.0,0.0,0.0,t61+t42.*t47.*t50.*x_ses13,t54-t44.*t47.*t50.*x_ses13,t59-t45.*t47.*t50.*x_ses13,t52-t46.*t47.*t50.*x_ses13,x_ses12.*(-1.0./5.0e+2),x_ses11./3.2e+2,9.999999975e-1],[13,13]);
