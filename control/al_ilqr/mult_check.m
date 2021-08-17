function mult = mult_check(con,mult,tol)

mult.mudx = check_con(con.cx,mult.lamx,mult.mux,tol);
mult.mudu = check_con(con.cu,mult.lamu,mult.muu,tol);

end