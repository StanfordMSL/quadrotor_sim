function obj = race_update(t_now,x_now)

% Load Circuit
stadium = load('scenarios/circuits/stadium.mat');
circuit = stadium.circuit;

% Initialize objective structure
obj.type = 'race';
obj.kf.x = zeros(13,2);

% Update Keyframes
if nargin == 1
  x_now = [ circuit.X(:,1) ; zeros(3,1)];
end

obj.kf.x(:,1) = x_now;

t_end = t_now+5.0;
if t_end < circuit.T
    k_kf  = round(t_end/circuit.dt);
    obj.kf.x(:,2) = [circuit.X(:,k_kf) ; zeros(3,1)];
else
    obj.kf.x(:,2) = [circuit.X(:,end) ; zeros(3,1)];
end

% Update Gate
% if (t_end < circuit.T/2)
%     gate = load('scenarios/circuits/gate2.mat');
% else
%     gate = load('scenarios/circuits/gate1.mat');
% end
% 
% obj.gt = gate.gate;
obj.gt.p_ctr = zeros(3,0);
obj.gt.p_box = zeros(3,4,0);
obj.gt.q_box = zeros(4,0);
obj.gt.seq  = zeros(1,0);
    
% Update Map
obj.map = [
    -8.1 8.1;       % Map x-limits (length)
    -3.2 3.2;       % Map y-limits (width)
     0 3];          % Map z-limits (height)      
 
end
