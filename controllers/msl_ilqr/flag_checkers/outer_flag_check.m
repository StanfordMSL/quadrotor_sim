function outer_flag = outer_flag_check(con,tol,itrs,itrs_max)

    % Constraint Trigger
    con_check_room  = sum(any(con(1:6,:) > tol));
    con_check_gates = sum(any(con(7:22,:) > tol));
    con_check_input = sum(any(con(23:30,:) > tol));
    con_status = [con_check_room con_check_gates con_check_input];

    if sum(con_status) > 0
        outer_flag = true;
    else
        outer_flag = false;
    end
    disp(['[con_flag_check]: Unsatisfied Constraints [room/gate/motor]: ',mat2str(con_status)]);

    % Iteration Trigger
    if itrs > itrs_max
        disp('[con_flag_check]: MAX ITERATIONS REACHED.');
        outer_flag = false;
    end
end