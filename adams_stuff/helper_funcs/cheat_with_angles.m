function q_hat = cheat_with_angles(q_hat)
    global yukf flight k
    
    [roll, pitch, yaw] = quat2angle(q_hat(:)', 'XYZ');
    [roll_act, pitch_act, yaw_act] = quat2angle(flight.x_act(7:10, k)', 'XYZ');
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
    q_hat = angle2quat(roll, pitch, yaw, 'XYZ')';
end
