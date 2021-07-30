function flag = check_outer(con,tol_motor,tol_gate)
    flag_gate  = sum(any(con.cx > tol_gate));
    flag_motor = sum(any(con.cu > tol_motor));

    con_status = [flag_motor flag_gate];
    
    if sum(con_status) > 0
        flag = false;       % keep going
    else
        flag = true;        % stop
    end
    
%     disp(['[check_outer]: Unsatisfied Constraints [motor/gate]: ',mat2str(con_status)]);

end