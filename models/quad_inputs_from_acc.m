function [u] = quad_inputs_from_acc(a_z,omega_dot,omega,model)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    I = model.I_act;
    inv_I = model.inv_I_act;
    L = model.L_act;
    b = model.b_act;    
    k2 = model.kt_act(1,1);
    k1 = model.kt_act(2,1);
    k0 = model.kt_act(3,1);

    tau = I*omega_dot + cross(omega,I*omega')';

    M = [1 1 1 1; -L L L -L; -L L -L L; -b -b b b];
    M_inv = 0.25*[1,-1/L,-1/L,-1/b ; 1,1/L,1/L,-1/b ; 1,1/L,-1/L,1/b; 1, -1/L,1/L,1/b];
    f = M_inv*[a_z;tau];
    % Assumes k1 and k0 are 0
    u = sqrt(f/k2);
end

