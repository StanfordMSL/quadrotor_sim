function m_cmd = wrench2m_controller(u,model)

m_cmd = zeros(4,1);

k1 = model.kt_est(1,1);
k2 = model.kt_est(2,1);
k3 = model.kt_est(3,1);

m_force = lsqnonneg(model.m2w,u(1:4,1));
% m_force = model.m2w_inv *u(1:4,1);

for k = 1:4
    interior = k2^2-(4*k1*(k3-m_force(k,1)));
    
    if interior < 0
        m_cmd(k,1) = 0;     % Command not feasible, zero-ing the motor command.
    else
        m_cmd(k,1) = (-k2 + sqrt(interior))/(2*k1);
    end
end

end