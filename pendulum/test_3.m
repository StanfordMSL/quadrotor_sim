clear 

% Define Model Parameters and Generate Corresponding System
model.dt = 0.01;
model.g = 9.81;
model.m = 0.1;
model.l = 1.5;

dyn_gen(model)

% Lock RNG
rng('default');

% Fixed Parameters
Nsim = 200;                         % Number of steps in trajectory 
Nvar = 1000;                        % Number of variations of initial conditions/parameter tests
s0  = [1.2 ; 0.0 ; 0.5 ; 1.0];      % Warm Start (guess) [ x0 ; th ]

%% Test Across Random Initial Conditions

Jout = zeros(2,Nvar);
Tout = zeros(2,Nvar);
Nout = zeros(2,Nvar);

for k = 1:Nvar
    % Generate Ground Truth and Sensor Data
    x0 = [-pi + 2.*pi.*rand() ; -10.0 + 20.0.*rand()];
    th = [rand() ; 1.0 + rand()];
    [Xact,Xdat] = data_gen(Nsim,x0,th);
    
    % Parameter Estimation
    [s,J,Tcomp,Nitr] = param_est(Xact,Xdat,s0);
    
    % Precision Trick
    if (J(1) >= 100) && (J(2) < 100)
        [s,J,Tcomp,Nitr] = param_est(Xact,Xdat,s(:,2));
    end
    
    Jout(:,k) = J';
    Tout(:,k) = Tcomp';
    Nout(:,k) = Nitr';
end

%% Process the Data
Jproc = Jout(:,all(Jout<100));
Tproc = Tout(:,all(Jout<100));
Nproc = Nout(:,all(Jout<100));

score = zeros(1,4);
for k = 1:Nvar
    if (Jout(1,k) < 100) && (Jout(2,k) >= 100)
        score(1,1) = score(1,1) + 1;        % comp wins
    elseif (Jout(1,k) >= 100) && (Jout(2,k) < 100)
        score(1,2) = score(1,2) + 1;        % indv wins
    elseif (Jout(1,k) < 100) && (Jout(2,k) < 100)
        score(1,3) = score(1,3) + 1;        % both win
    else
        score(1,4) = score(1,4) + 1;        % both lose
    end
end

score_normalized = score./Nvar;

%% Plot Performance
figure(1)
clf

subplot(4,1,1)
bins = categorical({'Comp1 Indv0','Comp0 Indv1','Comp1 Indv1','Comp0 Indv0'});
bins = reordercats(bins,{'Comp1 Indv0','Comp0 Indv1','Comp1 Indv1','Comp0 Indv0'});
bar(bins,score_normalized);

ylim([0.0 1.0]);
title('Robustness');

subplot(4,1,2)
plot(Jproc(1,:));
hold on
plot(Jproc(2,:));
legend('composite','individual')
title('Comp1 Indv1, Costs');

subplot(4,1,3)
plot(Tproc(1,:));
hold on
plot(Tproc(2,:));
legend('composite','individual')
title('Comp1 Indv1, Compute Time');

subplot(4,1,4)
plot(Nproc(1,:));
hold on
plot(Nproc(2,:));
legend('composite','individual')
title('Comp1 Indv1, Iterations');
