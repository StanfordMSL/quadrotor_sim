function qp_init(n_der)

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms t real
vect_basis = sym(zeros(1,n_der));
for k = 1:n_der
    vect_basis(1,k) = t^(k-1);
end

A_basis = sym(zeros(1,n_der));
for k = 1:n_der
    A_basis(k,:) = diff(vect_basis,t,(k-1));
end

syms t0 t1 real
H_r = int((A_basis(5,:)'*A_basis(5,:)),t,t0,t1);
H_psi = int((A_basis(3,:)'*A_basis(3,:)),t,t0,t1);

%% Output Files

matlabFunction(A_basis,'File','controllers/high_level/diff_flat/pre_compute/QP_A_basis')
matlabFunction(H_r,'File','controllers/high_level/diff_flat/pre_compute/QP_H_r')
matlabFunction(H_psi,'File','controllers/high_level/diff_flat/pre_compute/QP_H_psi')

disp("[dyn_init]: Basis Generated")
