clear

data = csvread('misc/actual_flight_data/MotorEstimates01/f1507.csv',1,0);

Aw2 = data(:,1).^2;
b   = data(:,2);

x = Aw2\b