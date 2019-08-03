function m_cmd = wrench2m_cmd(u,model)

m_cmd = zeros(4,1);

k1 = model.kt_est(1,1);
k2 = model.kt_est(2,1);
k3 = model.kt_est(3,1);

m_force = lsqnonneg(model.wrench,u);

for k = 1:4
    m_cmd(k,1) = (-k2 + sqrt(k2^2-(4*k1*(k3-m_force(k,1)))))/(2*k1);
end