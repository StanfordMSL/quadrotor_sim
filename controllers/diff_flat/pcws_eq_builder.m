function [Aeq,beq] = pcws_eq_builder(k_der,n_der,t_vect,sigma)

Aeq = zeros(k_der+1,n_der);
beq = zeros(k_der,1);

for k = 1:k_der+1
    idx1 = n_der-k+1;
    idx2 = k-1;

    t_feed  = [zeros(idx2,1)  ; t_vect(1:idx1,1)];

    Aeq(k,:) = t_feed';
    beq(k,1) = sigma(1,k);
end