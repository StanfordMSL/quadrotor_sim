function u_mt = wrench2motor(u_w,model)

T_motor = model.w2m*u_w;
u_mt = sqrt(T_motor./model.kw(1,1));

% Zero Any Impossible Forces
u_mt(imag(u_mt) ~= 0) = 0;