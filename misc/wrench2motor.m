function u_m = wrench2motor(u_w,model)

T_motor = model.w2m*u_w;
u_m = sqrt(T_motor./model.kw(1,1));
