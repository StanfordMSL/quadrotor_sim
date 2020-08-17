function [con_flag, con_status] = con_check(al,tol)

    con_check_input = sum(any(al.con(1:8,:) > tol));
    con_check_gates = sum(any(al.con(9:16,:) > tol));
    con_check_rates = sum(any(al.con(17:22,:) > tol));

    con_status = [con_check_input con_check_gates con_check_rates];

    if sum(con_status) > 0
        con_flag = true;
    else
        con_flag = false;
    end

end