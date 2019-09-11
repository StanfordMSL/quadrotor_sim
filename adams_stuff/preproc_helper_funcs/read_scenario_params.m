function prm = read_scenario_params(run_dir, scenario)

%     run_name = 'run1'; date_str = '20190309'; date_time_str = '2019-03-09-12-19-21'; camera_scenario = 2;   % training dataset
%     run_name = 'run2'; date_str = '20190309'; date_time_str = '2019-03-09-12-14-29'; camera_scenario = 3;   % training dataset
%     run_name = 'run3'; %date_str = '20190910'; date_time_str = '2019-09-10-12-50-30'; camera_scenario = 4;   % gentle motion
%     run_name = 'run4'; date_str = '20190910'; date_time_str = '2019-09-10-16-58-50';camera_scenario = 5;   % sharp motion
%     run_name = 'run5'; date_str = '20190910'; date_time_str = '2019-09-10-17-01-41'; camera_scenario = 5;   % gentle motion w/ yaw rotation


    fn = sprintf('%s/run%d_params.txt', run_dir, scenario);
    fid = fopen(fn,'rt');
    line = textscan(fid,'%s','Delimiter','\n');
    
    output = strsplit(line{1}{1}, ' ');
    prm.run_num = str2double(output{3});
    if prm.run_num ~= scenario
        error("Run # in parameter file does not match!")
    end
    
    output = strsplit(line{1}{2}, ' ');
    prm.date_str = output{3};
    
    output = strsplit(line{1}{3}, ' ');
    prm.datetime_str = output{3};
    
    output = strsplit(line{1}{4}, ' ');
    prm.camera_scenario = str2double(output{3});
    
    output = strsplit(line{1}{5}, ' ');
    prm.camera_pos_w = [str2double(output{3}), str2double(output{4}), str2double(output{5})];
    
    output = strsplit(line{1}{6}, ' ');
    prm.camera_quat_w = [str2double(output{3}), str2double(output{4}), str2double(output{5}), str2double(output{6})];
    
    output = strsplit(line{1}{7}, ' ');
    prm.camera_shift_w = [str2double(output{3}), str2double(output{4}), str2double(output{5})];
    
    output = strsplit(line{1}{8}, ' ');
    prm.camera_rot_x = str2double(output{3});
    
    output = strsplit(line{1}{9}, ' ');
    prm.camera_rot_y = str2double(output{3});
    
    output = strsplit(line{1}{10}, ' ');
    prm.camera_rot_z = str2double(output{3});
    
    output = strsplit(line{1}{11}, ' ');
    prm.quad_shift_w = [str2double(output{3}), str2double(output{4}), str2double(output{5})];
    
    fclose(fid);
end