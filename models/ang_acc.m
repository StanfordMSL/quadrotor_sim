function omega_dot = ang_acc(f,I,inv_I,omega,tau_ext,m2w)   

    m2w_tau = m2w(2:4,:);
    
    tau = (m2w_tau*f) + tau_ext;

    omega_dot = inv_I*(tau - cross(omega,I*omega));
end