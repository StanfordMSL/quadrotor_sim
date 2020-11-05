function [con,con_x,con_u] = gate_con(x_bar,obj,model)
%%  Generate points of interest
% We track 4 points on a cross-shaped quadcopter with a clockwise
% convention starting from the fore arm. We are looking to constrain
% the bounds into a gate using a basis vector + normal definition of
% the gate. gamma(1->2)[idx:1] and beta(1->4)[idx:2].

% Where applicable, subscript small letter variables are in body frame.
% Subscript caps are in world frame.

%% Unpack some stuff
% Count
N = size(x_bar,2);
n_c = 16;
n_x = size(x_bar,1);
n_u = 4;

% Gate Parameters
p_G1 = obj.p_gc(:,1);
p_G2 = obj.p_gc(:,2);
p_G4 = obj.p_gc(:,4);

r_12 = p_G2 - p_G1;
r_14 = p_G4 - p_G1;
r_BAS = [r_12 r_14];

n_G = cross(r_12,r_14);

% Quadcopter Dimensions
l_arm  = model.L_est;
dt_fmu = model.dt_fmu;

% Relative position of the four points to body center in body frame
r_b = [ l_arm    0.00  -l_arm    0.00;
         0.00  -l_arm    0.00   l_arm; 
         0.00    0.00    0.00    0.00 ];

% Initialize Output
con = zeros(16,N);
con_x = zeros(n_c,n_x,N);
con_u = zeros(n_c,n_u,N);

for k = 1:N
    % State Parameters
    p_B = x_bar(1:3,k);
    v_B = x_bar(4:6,k);
    q_b = x_bar(7:10,k);
    omega_b = x_bar(11:13,k);
    bRw = quat2rotm(q_b');

    % Compute the constraints relative to each point on the quadcopter
    for j = 1:size(r_b,2)
        % Relative Position of Point in World Frame
        r_arm = r_b(:,j);
        r_ARM = bRw*r_arm;

        % Global Position and Velocity of the Point in World Frame
        p_P = p_B + r_ARM;
        v_P = v_B + cross(omega_b,r_ARM);

        % Check if we are at the right frame by computing alpha_c
        den = dot(v_P,n_G);

        if den == 0
            % never hitting
        else
            alpha_c = dot((p_G1-p_P),n_G)/den;

            if (alpha_c > 0) && (alpha_c < 2*dt_fmu)
                % Contact point
                p_C   = p_P + (alpha_c.* v_P);    % absolute
                r_G1C = p_C - p_G1;               % relative to g1

                % For each basis vector
                for b = 1:2
                    idx = 4*(j-1)+2*(b-1)+1;

                    [con(idx:idx+1,k),con_x(idx:idx+1,:,k)] = basis_con(alpha_c,r_G1C,r_BAS(:,b),r_arm,x_bar);
                end
            else
                % Pass through. no contact.
            end
        end

        
    end
end

end