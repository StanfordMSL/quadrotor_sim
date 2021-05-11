function precompute(model,n_der)   

% Generate Dynamic Functions (Jacobian and Hessians)
dyn_init(model,'act');
dyn_init(model,'est');

% Generate QP Matrices
qp_init(n_der);