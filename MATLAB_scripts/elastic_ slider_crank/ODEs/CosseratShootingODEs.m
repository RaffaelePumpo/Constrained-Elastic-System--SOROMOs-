function dydx = CosseratShootingODEs(~, y, Const)
% The state has the form
%     | r |   x, y, z                      1-3
%     | Q |   w, x, y, z                   4-7
%     | m |  mx, my, mz                    8-10
%     | f |  fx, fy, fz                   11-13

%   Get needed quatities
Area = Const.Area;
g = Const.g;
rho = Const.rho;
H = Const.H;
Xi_c = Const.Xi_c;

%   Decompose state vector
Q = y(4:7);
R = Q2R(Q);
m = y(8:10);
f = y(11:13);


%   Project forces in local coordinates
N = R'*f;
C = R'*m;
Lambda = [C; N];


%   Compute the strains
Xi = inv(H)*Lambda + Xi_c;
K = Xi(1:3);
Gamma = Xi(4:6);

%   Gravitational force
f_bar = [0; 0; rho*Area*g];

%   ODEs
r_prime =  R*Gamma;
Q_prime =  0.5*getA(K)*Q;
f_prime =  f_bar;
m_prime = -hat(r_prime)*f;

dydx = Const.L*[r_prime; Q_prime; m_prime; f_prime];

end