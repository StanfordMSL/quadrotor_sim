clear

% syms qw qx qy qz real
% q   = [qw ;  qx ;  qy ;  qz];
% 
% syms c real
% thrust = [ 0 ; 0 ; c];
% 
% syms dx dy dz real
% Dx = [ dx ; 0  ; 0];
% Dy = [ 0  ; dy ; 0];
% Dz = [ 0  ; 0  ; dz];
% 
% syms vx vy vz real
% v = [vx ; vy ; vz];
% 
% thurst_vect = quatrot2(thrust,q)
% drag_vect   = quatrot2(Dx,q) + quatrot2(Dy,q) + quatrot2(Dz,q)

syms dt m g real

syms qw qx qy qz real
q   = [qw ;  qx ;  qy ;  qz];
q_c = [qw ; -qx ; -qy ; -qz];

syms Jxx Jyy Jzz real
J = diag([Jxx Jyy Jzz]);

syms L b real
M = [ 1  1  1  1;...
     -L  L  L -L;...
     -L  L -L  L;...
     -b -b  b  b];  

syms wx wy wz real
w = [wx ; wy ; wz];

syms wx_p wy_p wz_p real
w_p = [wx_p ; wy_p ; wz_p];

syms vx vy vz real
v = [vx ; vy ; vz];

syms vx_p vy_p vz_p real
v_p = [vx_p ; vy_p ; vz_p];

c1 = (m/dt)*(v-v_p) - [0 ; 0 ; m*g];
c2 = quatrot2(c1,q_c);

% temps = subs(c2,{qw qx qy qz}, {sqrt(0.5),0,sqrt(0.5),0})

q_1 = [0.9224 ; -0.2745 ; -0.2671 ; 0.0504];
v_2_act = [1.9329 ; 0.4274 ; 0.0517];
v_1_act = [2.0048 ; 0.3610 ; 0.0520];
m_act = 0.6500;
g_act = -9.8100;
dt_act = 0.01;
c_sim = 8.9897;
c_out = [ 0 ; 0 ; c_sim];

c2sa = subs(c2,{qw qx qy qz}, {q_1(1),q_1(2),q_1(3),q_1(4)});
c2sb = subs(c2sa,{vx_p vy_p vz_p vx vy vz}, {v_1_act(1),v_1_act(2),v_1_act(3),v_2_act(1),v_2_act(2),v_2_act(3)});
c2sc = subs(c2sb,{m,g,dt}, {m_act,g_act,dt_act});
compare_1 = [c_out double(c2sc)]

c3 = (dt/m)*([0 ; 0 ; m*g] + quatrot2([0 ; 0 ; c_sim],q));
c3sa = subs(c3,{qw qx qy qz}, {q_1(1),q_1(2),q_1(3),q_1(4)});
c3sb = subs(c3sa,{m,g,dt}, {m_act,g_act,dt_act});
compare_2 = [v_2_act-v_1_act double(c3sb)]

% c = temp(3,1);
