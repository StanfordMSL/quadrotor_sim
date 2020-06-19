clear

x = -1:0.001:1

a = 0.001;
term = x./a;

dirac = exp(-term.^2);

figure(1)
clf
plot(x,dirac,'*')