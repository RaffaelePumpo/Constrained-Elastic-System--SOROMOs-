function [r_X1, Q_X1, Qa_X0] = IGM(chi, Config, Const)

%%                          INITIALIZATION
%   Extract needed quantities
ne = Const.dim_base;
qr = Const.qr;
Q_0 = Const.Q_0;
r_0 = Const.r_0;
R_0 = Q2R(Q_0);


%   [TODO]  Decompose state vector in order to take the out the generalized
%   coordinates and the other variables
q = chi(1:ne);
r_base = [Const.r_base 0 0]';

%   We can get the rotation matriz associated with a rotation along the z
%   axis of an agle theta (remember that the rod is attached to
%   the joint so its initial pose will be the position of the joint plus
%   the rotation of the joint axis)
Rz = rotZ(qr);

%   [TODO]  Compute the orientation of the rod base (X=0) with respect to
%   the absolute frame
R_X0 = Rz*R_0;
Q_X0 = R2Q(R_X0);

%   [TODO]  Compute the position of the rod base (X=0) with respect to
%   the absolute frame
r_X0 = zeros(3, 1);
r_X0 = r_0-r_base;

%%                          Forward ODEs
%   Initialization
forward_y0 = [Q_X0;
              r_X0];

%   [TODO]  Forward integration
[~, Y] = ode45(@(X, y) ForwardKinematics(X, y, q, Const),Config.forward_integration_domain, forward_y0);
%   [TODO]  Extract state at X=1
fk_y_X1 = Y(end,:);
Q_X1 = fk_y_X1(1:4)';
r_X1 = fk_y_X1(5:7)';
R_X1 = Q2R(Q_X1);

%%                          Backward ODEs
%   [TODO]  Force at the tip. Now evaluate the force at the rod tip. We use
%   the full se(3) wrench which will mostly filled with zeros. Howver, the
%   joint might impose some efforts on the rod tip. Their expression is
%   contained in the chi vector, being part of the unknowns of the problem
F1 = [0;0;chi(ne+2);0;chi(ne+1);0];



projection_matrix = [  R_X1  , zeros(3);
                     zeros(3),   R_X1  ];

%   [TODO]  Project wrench into local coordinates
Lambda_X1 = projection_matrix' * F1;

%   Initialization
backward_y1 = [ Q_X1;
                r_X1;
                Lambda_X1;
                zeros(ne, 1)];


%   [TODO]  Backward integration
[~, Y] = ode45(@(X, y) BackwardStatics(X, y, q, Const),Config.backward_integration_domain, backward_y1);
%   [TODO]  Extract state at X=0
backward_yX = Y(end,:);
%   [TODO]  Extract Qa_X0 from the state of the rod base (at X=0)
Qa_X0 = backward_yX(14:end)';
end

