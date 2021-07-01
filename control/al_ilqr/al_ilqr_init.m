function al_ilqr_init(cost_mode,input_mode,obj,map,model)

% Initialize the Cost Functions
lagr_init(cost_mode,input_mode,obj)

% % Initialize the  Contstraint Functions
motor_con_init(input_mode,model)
gate_con_init(map,input_mode,model)

end