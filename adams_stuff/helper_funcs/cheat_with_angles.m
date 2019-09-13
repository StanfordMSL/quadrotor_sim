function q_hat = cheat_with_angles(q_hat)
    global yukf flight k
    
    [yaw, pitch, roll] = quat2angle(q_hat(:)');
    [yaw_act, pitch_act, roll_act] = quat2angle(complete_unit_quat(flight.x_act(7:9, k))');
    if yukf.prms.b_enforce_yaw
        yaw = yaw_act;
    end
    if yukf.prms.b_enforce_0_yaw
        yaw = 0;
    end
    if yukf.prms.b_enforce_pitch
        pitch = pitch_act;
    end
    if yukf.prms.b_enforce_roll
        roll = roll_act;
    end
    q_hat = angle2quat(yaw, pitch, roll)';
end



        
%         quat_act = complete_unit_quat(flight.x_act(7:9, k_act));
%         [yaw_act, pitch_act, roll_act] = quat2angle(quat_act(:)');
%         if yukf.prms.b_enforce_0_yaw
%             mu_out(9) = 0; % can probably do a better job forcing yaw to 0 (convert to eul, zero yaw, convert back)
%         end
%         if yukf.prms.b_enforce_0_pitch
%             mu_out(8) = pitch_act; % can probably do a better job forcing yaw to 0 (convert to eul, zero yaw, convert back)
%         end
%         if yukf.prms.b_enforce_0_roll
%             mu_out(7) = roll_act; % can probably do a better job forcing yaw to 0 (convert to eul, zero yaw, convert back)
%         end