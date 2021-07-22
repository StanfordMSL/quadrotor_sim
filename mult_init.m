function mult = mult_init(X,U,xs,us,T,c)

N = size(X,2);

mu = [ 1.*ones(8,N) ;...   % motor
       1.*ones(16,N) ];    % gate
lambda = 0.*ones(24,N);

mu_d = check_con(c,lambda,mu,0);
La_c = lagr_calc(X,U,xs,us,T,c,lambda,mu_d);

coeff = La_c.obj/La_c.con;

mult.mu     = coeff*mu;
mult.mu_d   = check_con(c,lambda,mult.mu,0);
mult.lambda = lambda;
mult.phi    = 1.1;

