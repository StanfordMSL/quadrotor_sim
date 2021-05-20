function outer_flag = outer_flag_check(con,tol_pos,tol_mot,itrs,itrs_max)

    % Constraint Trigger
    con_check_gates = sum(any(con(1:16,:) > tol_pos));
    con_check_input = sum(any(con(17:24,:) > tol_mot));

    con_status = [con_check_gates con_check_input];

    if sum(con_status) > 0
        outer_flag = true;
    else
        outer_flag = false;
    end
    disp(['[outer_flag_check]: Unsatisfied Constraints [gate/motor]: ',mat2str(con_status)]);

    % Iteration Trigger
    if itrs > itrs_max
        disp('[outer_flag_check]: MAX ITERATIONS REACHED.');
        outer_flag = false;
    end
end