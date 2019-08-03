function FT_ext = nudge(FT,curr_time,nudge_time,nudge_duration)
    if (curr_time > nudge_time) && (curr_time < nudge_time + nudge_duration) 
        FT_ext = FT;
    else
        FT_ext = [0 0 0 0 0 0]';
    end
end