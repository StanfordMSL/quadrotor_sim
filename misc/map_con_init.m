function map_con_init(map)

tic

% Variable dimensions based on Input
switch input_mode
    case 'direct'
        n_s = 13;
    case 'wrench'
        n_s = 13;
    case 'body_rate'
        n_s = 17;
end

s = sym('s',[n_s 1],'real');
x_lim = sym('x_lim',[1 2],'real');
y_lim = sym('y_lim',[1 2],'real');
z_lim = sym('z_lim',[1 2],'real');

s(1) = 
disp(['[gate_con_init]: Map Constraints Generated in ' num2str(t_comp) 's'])