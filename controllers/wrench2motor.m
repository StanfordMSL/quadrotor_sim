function u_m = wrench2motor(u_w,model)

T_motor = model.m2w_est_inv*u_w;
u_m = sqrt(T_motor./model.kw_est(1,1));
