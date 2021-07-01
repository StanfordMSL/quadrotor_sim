function [Aeq, beq] = QP_con_builder(t_pw,sigma_pw,con_pw)

n_con = sum(con_pw,'all');

A_temp = QP_A_basis(0);
n_der = size(A_temp,1);

Aeq = zeros(n_con,n_der);
beq = zeros(n_con,1);

idx = 1;
for k_t = 1:2
    A_basis = QP_A_basis(t_pw(k_t));

    for k_der = 1:size(con_pw,1)
        if con_pw(k_der,k_t) == 1
            Aeq(idx,:) = A_basis(k_der,:);
            beq(idx,1) = sigma_pw(k_der,k_t);
            idx = idx + 1;
        end
    end
end

end

