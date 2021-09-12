function mult = mult_init(con)

% Unpack Some Stuff
[nx,N] = size(con.cx);
nu     = size(con.cu,1);

% Initialize Multiplier Terms
mult.lamx = 0.*ones(nx,N);
mult.lamu = 0.*ones(nu,N);

mult.mux  = (1e-3)*ones(nx,N);
mult.muu  = (1e-3)*ones(nu,N);
mult.mudx = zeros(nx,N);
mult.mudu = zeros(nu,N);

% Run the check once
mult = mult_check(con,mult,0);