%% Environment Setup (MATLAB paths and CASADI) %%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; 
addpath(genpath(pwd));
addpath('/home/lowjunen/casadi-linux-matlabR2014b-v3.5.5')
opti = casadi.Opti();

%% Load Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

model = model_init('carlito','match','precise');

quad2Dsym(model)

%% Generate Initial Solution

%% Initial Solution
[Xs,Us] = NLP_solve(model);

%% Sample Test

% Generate Backward Pass
[l,Ls,delV] = backward_pass(Xs,Us);

% Time Variable
N = size(Us,2);
dt = model.est.dt;
T = 0:dt:N*dt;

%% Sample Test
traj_plot(Xs);
hold on
N_test = size((1:10:N),2);
succ = zeros(3,N_test);
counter = 1;
for j = 1:10:N
    Xbar = Xs(:,j:end);
    Ubar = Us(:,j:end);
    Lbar = Ls(:,:,j:end);
    
    xj = Xbar(:,1);
    [X1,X2,X3,X4,X5,X6,Ns] = mesh_gen(xj);

    x_g = zeros(6,0);
    x_b = zeros(6,0);
    err = zeros(1,Ns);
    for k = 1:Ns
        x0 = [X1(k) ; X2(k) ; X3(k) ; X4(k) ; X5(k) ; X6(k)];
        err(1,k) = forward_pass(Xbar,Ubar,Lbar,x0,dt);
        if err(1,k) < 0.1
            x_g = [x_g x0];
        else
            x_b = [x_b x0];    
        end
    end
    plot(x_b(1,:),x_b(2,:),'r*')
    plot(x_g(1,:),x_g(2,:),'bo')
    
    ratio = size(x_g,2)/(size(x_b,2)+size(x_g,2));
    succ(1,counter) = size(x_g,2);
    succ(2,counter) = size(x_b,2);
    succ(3,counter) = ratio;
        
    counter = counter + 1;
end

%% Jacobian and Hessian

sens = zeros(1,N_test);
counter = 1;
for j = 1:10:N
    x = Xs(:,j);
    u = Us(:,j);
    
    H = zeros(6,6,6);
    H(:,:,1) = H1_calc(x,u);
    H(:,:,2) = H2_calc(x,u);
    H(:,:,3) = H3_calc(x,u);
    H(:,:,4) = H4_calc(x,u);   
    H(:,:,5) = H5_calc(x,u);
    H(:,:,6) = H6_calc(x,u);
    
    x_test = zeros(6,1);
    for k = 1:6
        xk = Xs(:,j+1);
        x_test(k,1) = xk'*H(:,:,k)*xk;
    end
    
    sens(1,counter) = norm(x_test);
    counter = counter + 1;
end

figure(4)
clf
plot(sens./100,'*--')
hold on
plot(succ(3,:),'o--')
legend('Hessian Sensitivity','Success Rate')
% figure(1)
% clf
% subplot(2,1,1)
% plot(T,Xbar(1,:));
% hold on
% plot(T,Xbar(1,:));
% legend('ideal','simulated');
% 
% subplot(2,1,2)
% plot(T,Xbar(2,:));
% hold on
% plot(T,Xbar(2,:));
% legend('ideal','simulated');

