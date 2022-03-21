function model = param_gen(file,type)

address = ['airframes/',file];

% Read CSV file
data = readtable(address);

% Unpack CSV file
model.act.m   = data{2,2};
model.act.Ipp = data{3,2:4}';
model.act.Ipd = data{4,2:4}';
model.act.g   = data{5,2};

model.act.Df = data{7:9,2:4};

model.act.kw = data{11,2};

b  = data{12,2};
L = data{13:16,2:4};
model.act.fm2wr = [...
     1.0     1.0    1.0    1.0  ;...
    L(1,2)  L(2,2) L(3,1) L(4,1);...
    L(1,1)  L(1,2) L(1,1) L(1,1);...
     -b      -b      b      b];

model.est = model.act;

switch type
    case 'mismatch'
        model.est.m = 1.005*model.act.m;
        model.est.Ipp(1:3,1) = 0.0010;
    case 'match'
        % Do Nothing
end