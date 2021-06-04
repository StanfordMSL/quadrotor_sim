function flag = check_outer(con,tol_motor,tol_gate)
    flag_motor = sum(any(con(1:8,:) > tol_motor));
    flag_gate  = sum(any(con(9:24,:) > tol_gate));
    
    con_status = [flag_motor flag_gate];
    
    if sum(con_status) > 0
        flag = false;       % keep going
    else
        flag = true;        % stop
    end
    
%     disp(['[check_outer]: Unsatisfied Constraints [motor/gate]: ',mat2str(con_status)]);

end