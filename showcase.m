clear
addpath(genpath(pwd));

pause(2)

load flight_zigzag.mat
load wp_zigzag.mat 
animation_plot(flight,wp,'persp');

pause(1)

load flight_square.mat
load wp_square.mat 
animation_plot(flight,wp,'persp');

pause(1)

load flight_circle.mat
load wp_circle.mat 
animation_plot(flight,wp,'persp');

pause(1)

load flight_slit.mat
load wp_slit.mat 
animation_plot(flight,wp,'persp');

pause(1)

load flight_targeted.mat
load wp_targeted.mat 
animation_plot(flight,wp,'persp');
pause(1)
animation_plot(flight,wp,'side');
