function omega_dot = ang_acc(f,I,inv_I,L,b,omega,tau_ext,tau_yaw)    
    tau = [L*(f(2) + f(3) - f(1) - f(4)) ;...
           L*(f(2) + f(4) - f(1) - f(3)) ;...
           b*(f(3) + f(4) - f(1) - f(2))+tau_yaw];

    tau = tau + tau_ext;

    omega_dot = inv_I*(tau - cross(omega,I*omega));
end