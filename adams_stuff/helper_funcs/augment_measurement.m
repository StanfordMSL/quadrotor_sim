function output_modified = augment_measurement(output, yukf, state_gt)
    output_modified = output(:);
    if yukf.prms.b_measure_everything
        warning("cheating by measuring full state perfectly!!")
        output_modified = state_gt;
    end
                
    if yukf.prms.b_measure_aspect_ratio
        output_modified = [output_modified; output_modified(3)/output_modified(4)];
    end
    [yaw_act, pitch_act, roll_act] = quat2angle(complete_unit_quat(state_gt(7:9))');
    if yukf.prms.b_measure_yaw
        if yukf.prms.b_enforce_0_yaw
            output_modified = [output_modified; 0];
        else
            output_modified = [output_modified; yaw_act];
        end
    end
    if yukf.prms.b_measure_pitch
        output_modified = [output_modified; pitch_act];
    end
    if yukf.prms.b_measure_roll
        output_modified = [output_modified; roll_act];
    end
end