function [dydx] = ForwardKinematics(X, y, q, Const)

% The state has the form
%     | Q |   w, x, y, z                  1-4
%     | r |   x, y, z                     5-7
%     | η |   Ω1 , Ω2 , Ω3 , V1 , V2 , V3 8-13
%     | η̇ |   Ω1 , Ω2 , Ω3 , V1 , V2 , V3 14-19

%   Obtain the need variables
B     = Const.B;
Xi_c  = Const.Xi_c;
L     = Const.L;


%   Compute strains
Phi = getPhi(X, Const);

Xi      = B*Phi*q + Xi_c;

K     = Xi(1:3);
Gamma = Xi(4:6);


%  Unpack state vector
Q       = y(1:4);


%   Compute the derivatives
Q_prime       =   1/2*getA(K)*Q;
r_prime       =   Q2R(Q)*Gamma;



%  Packing state vector derivative
dydx = L*[Q_prime;
          r_prime];
end