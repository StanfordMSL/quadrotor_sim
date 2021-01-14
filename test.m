clear

m = 0.5;
I = 0.001.*[  1.54   0.00   0.00;...
              0.00   1.54   0.00;...
              0.00   0.00   2.51]; 
inv_I = inv(I);
kd = 1;
D = [  0.10   0.00   0.00;...
       0.00   0.10   0.00;...
       0.00   0.00   0.30];
dt = 0.05;

t_old = 0;
t_new = 0;
for k = 1:100
    x_curr = rand(13,1);
    x_curr(7:10) = [0 ; 1 ; 0 ; 0];
    u_curr = 100.*rand(4,1);
    
    tic
    Ixx = I(1,1);
    Iyy = I(2,2);
    Izz = I(3,3);
    kA_x = 0.00001;
    kA_y = 0.00001;
    kA_z = 0.00001;
    kB_x = 0.00001;
    kB_y = 0.00001;
    kB_z = 0.00001;
    kD_x = D(1,1);
    kD_y = D(2,2);
    kD_z = D(3,3);
    k_h = 0.009*m;
    k_w = 8.8478e-09;
    qw = x_curr(7,1);
    qx = x_curr(8,1);
    qy = x_curr(9,1);
    qz = x_curr(10,1);
    u1 = u_curr(1,1);
    u2 = u_curr(2,1);
    u3 = u_curr(3,1);
    u4 = u_curr(4,1);
    vx = x_curr(4,1);
    vy = x_curr(5,1);
    vz = x_curr(6,1);
    wx = x_curr(11,1);
    wy = x_curr(12,1);
    wz = x_curr(13,1);

    temp2 = J_x_test(Ixx,Iyy,Izz,kA_x,kA_y,kA_z,kB_x,kB_y,kB_z,kD_x,kD_y,kD_z,k_h,k_w,m,qw,qx,qy,qz,u1,u2,u3,u4,vx,vy,vz,wx,wy,wz);
    t_new = t_new + toc;
    
    tic
    temp1 = A_calc(x_curr,u_curr,m,I,inv_I,kd,D,dt);
    t_old = t_old + toc;
end

final = [t_old t_new] 