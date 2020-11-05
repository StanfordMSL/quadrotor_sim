function [Aeq,beq] = pcws_eq_builder_v2(k_der,n_der,t_vect,sigma)

if k_der == 2
    Aeq = zeros(2,n_der);
    beq = zeros(2,1);
    
    Aeq(1,:) = t_vect';
    beq(1,1) = sigma(1,1);
    Aeq(2,:) = [0 ; 0 ; t_vect(1:end-2,1)]';
    beq(2,1) = sigma(1,3);
else
    Aeq = zeros(k_der+1,n_der);
    beq = zeros(k_der,1);
    
    for k = 1:k_der+1
        idx1 = n_der-k+1;
        idx2 = k-1;

        t_feed  = [zeros(idx2,1)  ; t_vect(1:idx1,1)];

        Aeq(k,:) = t_feed';
        beq(k,1) = sigma(1,k);
    end
end
