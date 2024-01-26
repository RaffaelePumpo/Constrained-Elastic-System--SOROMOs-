function [dydx] = BackwardStatics(X, y, q, Const)

% The state has the form
%     | Q |   w, x, y, z                  1-4
%     | r |   x, y, z                     5-7
%     | Λ |   C1, C2, C3, N1, N2, N3      8-13
%     | Qa|                               14-end


%   Obtain the need variables
B     = Const.B;
Xi_c  = Const.Xi_c;
L     = Const.L;



%   Compute strains
Phi = getPhi(X, Const);

Xi  = B*Phi*q + Xi_c;


%  Unpack state vector
Q       = y(1:4);
Lambda  = y(8:13);

%   Get the external force
F_bar = F_gravity(Q, Const);

%   Compute the derivatives
dydx_kinematics = ForwardKinematics(X, y, q, Const);
Lambda_prime    = ad(Xi)'*Lambda - F_bar;
Qa_prime        = -Phi'*B'*Lambda;

%  Packing state vector derivative
dydx = L*[dydx_kinematics;
          Lambda_prime;
          Qa_prime];


end