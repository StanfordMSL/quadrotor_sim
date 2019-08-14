function m_cmd = wrench2m_controller(u,model)

m_cmd = zeros(4,1);

k1 = model.kt_est(1,1);
k2 = model.kt_est(2,1);
k3 = model.kt_est(3,1);

m_force = lsqnonneg(model.wrench,u);

for k = 1:4
    m_cmd(k,1) = (-k2 + sqrt(k2^2-(4*k1*(k3-m_force(k,1)))))/(2*k1);
end

for k = 1:4
    if m_cmd(k,1) < model.motor_min
        m_cmd(k,1) = model.motor_min;

        if m_cmd(k,1) <  (model.motor_min - 1e-6)
            disp('[ilqr]: Exceeded Motor Min');
        end

    elseif m_cmd(k,1) > model.motor_max
        m_cmd(k,1) = model.motor_max;

        if m_cmd(k,1) > (model.motor_max + 1e-6)
            disp('[ilqr]: Exceeded Motor Max');
        end

    end
end