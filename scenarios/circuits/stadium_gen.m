function circuit = stadium_gen()

r = 1;
l_rect = 12;

% Assuming we are flying at around 1m/s.
hz = 10;
dt = 1/hz;
vel_c = 1;
pos0 = [-l_rect/2 ; r ; 1 ];

% Part 1
t_p1 = 0:dt:l_rect/vel_c;
N = size(t_p1,2);
posp1 = pos0 + [
    t_p1*vel_c;
    zeros(1,N);
    zeros(1,N)];
   
% Part 2
t_p2 = 0:dt:round(pi*r,2)/vel_c;
N = size(t_p2,2);
ang = linspace(pi/2,-pi/2,N);
posp2 = posp1(:,end) + [0 ; -1 ; 0] + [
    cos(ang) ;
    sin(ang) ;
    zeros(1,N)];

% Part 3
t_p1 = 0:dt:l_rect/vel_c;
N = size(t_p1,2);
posp3 = posp2(:,end) + [
    -t_p1*vel_c;
    zeros(1,N);
    zeros(1,N)];

% Part 4
t_p2 = 0:dt:round(pi*r,2)/vel_c;
N = size(t_p2,2);
ang = linspace(-pi/2,pi/2,N);
posp4 = posp3(:,end) + [0 ; 1 ; 0] + [
    -cos(ang) ;
    sin(ang) ;
    zeros(1,N)];

% Full Position
pos = [posp1(:,1:end-1) posp2(:,1:end-1) posp3(:,1:end-1) posp4(:,1:end-1)];

% Useful Term
N = size(pos,2);

% Corresponding quaternions
quat = zeros(4,N);
vect0 = [1 ; 0 ; 0];
quat(:,1) = [-1 0 0 0];
for k = 2:N
    vect1 = pos(:,k)-pos(:,k-1);
    r = vrrotvec(vect0,vect1);
    quat(:,k) = -axang2quat(r);
end

circuit.T = dt*(N-1);
circuit.dt = dt;
circuit.X = [pos ; zeros(3,N); quat];


end
