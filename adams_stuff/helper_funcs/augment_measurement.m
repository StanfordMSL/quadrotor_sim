function output_modified = augment_measurement(output, yukf, state_est, state_gt)
    output_modified = output(:);
    if yukf.prms.b_measure_everything
        warning("cheating by measuring full state perfectly!!")
        output_modified = state_gt;
    end
                
    if yukf.prms.b_measure_aspect_ratio
        output_modified = [output_modified; output_modified(3)/output_modified(4)];
    end
    
    [roll, pitch, yaw] = quat2angle(state_est(7:10)', 'XYZ');
    [roll_act, pitch_act, yaw_act] = quat2angle(state_gt(7:10)', 'XYZ');
    if yukf.prms.b_measure_yaw
        if yukf.prms.b_enforce_0_yaw
            output_modified = [output_modified; 0];
        end
    end
    
%     [roll, pitch, yaw] = quat2angle(state_est(7:10)', 'XYZ');
%     if yukf.prms.b_measure_pitch
%         output_modified = [output_modified; pitch];
%     end
end