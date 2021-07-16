function QP_init(n_der)

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms t real
syms c [n_der 1] real
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

Ad04 = A_basis(1:5,:);
Ad02 = A_basis(1:3,:);
Ad00 = A_basis(1,:);

%% Output Files

matlabFunction(H_r,'File','control/diff_flat/QP/H_r','vars',{t0,t1})
matlabFunction(H_psi,'File','control/diff_flat/QP/H_psi','vars',{t0,t1})
matlabFunction(Ad04,'File','control/diff_flat/QP/Ad04','vars',t)
matlabFunction(Ad02,'File','control/diff_flat/QP/Ad02','vars',t)
matlabFunction(Ad00,'File','control/diff_flat/QP/Ad00','vars',t)

disp("[QP_init]: Basis Generated")
