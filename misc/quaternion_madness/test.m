clear

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Time Step
% dt = 1/200;
% 
% % Generate Test Quaternion and Body Rate
% rng('default');
% q0 = zeros(4,1);
% [q0(1), q0(2), q0(3), q0(4)] = parts(randrot(1));
% w = [ 1 ; 3 ; 0];
% 
% % Intermediate Terms
% W = [  0  -w(1) -w(2) -w(3) ;
%      w(1)   0    w(3) -w(2) ;
%      w(2) -w(3)    0   w(1) ;
%      w(3)  w(2) -w(1)    0];                % Body Rate vee map
%  
% % Supposed Trick
% q = zeros(4,100);
% q(:,1) = q0;
% 
% for k = 1:599
%     q_dot = 0.5*W*q(:,k);
% 
%     q(:,k+1) = q(:,k) + dt.*q_dot;
% end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear
% 
% % Time Step
% dt = 1/200;
% 
% % State Variables
% q = sym('q',[4 1],'real');
% w = sym('w',[3 1],'real');
% 
% % Intermediate Terms
% W = [  0  -w(1) -w(2) -w(3) ;
%      w(1)   0    w(3) -w(2) ;
%      w(2) -w(3)    0   w(1) ;
%      w(3)  w(2) -w(1)    0];                % Body Rate vee map
%  
% % Supposed Trick
% % q_u = q./norm(q);
% q_u = q;
% 
% % Generate an Update
% q_dot = 0.5*W*q_u;
% q_upd = q_u + dt.*q_dot;
% 
% % Test with real values
% 
% % Generate Test Quaternion and Body Rate
% rng('default');
% q_t = zeros(4,1);
% [q_t(1), q_t(2), q_t(3), q_t(4)] = parts(randrot(1));
% w_t = [ 0 ; 3 ; 0];
% 
% % Substitute
% truth = subs(q_upd,q,q_t);
% truth = subs(truth,w,w_t);
% 
% % Moment of Truth
% truth = double(truth);
% out = norm(truth)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% qv1 = [ 0 ; 0.3 ; 0.1];
% qw1 = sqrt(1-qv1'*qv1);
% 
% q1_a = [qw1 ; qv1];
% q1_b = [-qw1 ; qv1];
% 
% % hope = [norm(q1_a) quat2eul(q1_a')]
% % faith = [norm(q1_b) quat2eul(q1_b')]
% hope = [norm(q1_a) quat2eul(q1_a')]
% faith = [norm(q1_b) quat2eul(quatconj(q1_b'))]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic
rng('default');
for k = 1:1000
   q = randn(4,1);
   q = q./norm(q);
end
toc

tic
rng('default');
for k = 1:1000
   q = randn(4,1);
   
   q_sq = q'*q;
   
   if (abs(1-q_sq) < 2.107342e-08)
       q = q*(2/1+q_sq);
   else
       q = q*(1/q_sq);
   end
end
toc