function [inputs, err] = pid_controller(time,state,model,err,x_nom)       
    % Compute Pos Errors
    err.P_pos = state(1:3,1) - x_nom(1:3,1);
    err.I_pos = err.I_pos + (err.P_pos * err.dt);
    err.D_pos = (err.P_pos - err.P_pos_prior)/err.dt;
    err.P_pos_prior = err.P_pos;
    
    % Anti-Wind Up
    if err.I_pos > err.I_pos_max
        err.I_pos(1:3,1) = err.I_pos_max;
    end
    
    % Compute Attitude Errors
    err.P_att = err.P_att + (time.fc_dt.*state(11:13,1)) - x_nom(10:12,1);
    err.I_att = err.I_att + (err.P_att * err.dt);
    err.D_att = (err.P_att - err.P_att_prior)/err.dt;
    err.P_att_prior = err.P_att;
    
    % Anti-Wind Up
    if err.I_att > err.I_att_max
        err.I_att(1:3,1) = err.I_att_max;
    end
    
    % Compute XY Commands
    pos_x_error = model.xykP*err.P_pos(1) + model.xykI*err.I_pos(1) + model.xykD*err.D_pos(1);
    pos_y_error = model.xykP*err.P_pos(2) + model.xykI*err.I_pos(2) + model.xykD*err.D_pos(2);
    
    if abs(pos_x_error) > err.xy_limit
        pos_x_error =  sign(pos_x_error)*err.xy_limit;
    end
    
    if abs(pos_y_error) > err.xy_limit
        pos_y_error =  sign(pos_y_error)*err.xy_limit;
    end
     
    pos_z_error = -model.zkP*err.P_pos(3) - model.zkI*err.I_pos(3) - model.zkD*err.D_pos(3);
   
    pos_z_inputs = pos_z_error;
    % Compute Attitude Commands
    att_xy_error = model.att_xy_kP*err.P_att(1:2,1) + model.att_xy_kI*err.I_att(1:2,1) + model.att_xy_kD*err.D_att(1:2,1);
    att_z_error =  model.att_z_kP*err.P_att(3,1) + model.att_z_kI*err.I_att(3,1) + model.att_z_kD*err.D_att(3,1);

    att_xy_error = att_xy_error + [-pos_y_error ; pos_x_error];
    att_error = [att_xy_error ; att_z_error];
    
    att_inputs(1,1) = -att_error(1,1) + att_error(2,1) + att_error(3,1);
    att_inputs(2,1) =  att_error(1,1) - att_error(2,1) + att_error(3,1);
    att_inputs(3,1) = -att_error(1,1) - att_error(2,1) - att_error(3,1);
    att_inputs(4,1) =  att_error(1,1) + att_error(2,1) - att_error(3,1);
    
    % Compute Total Commands
    inputs = pos_z_inputs + att_inputs;
    
    for k = 1:4
        if inputs(k,1) < 0
            inputs(k,1) = 0;
        end
        if inputs(k,1) > 2200
            inputs(k,1) = 2200;
        end
    end
end