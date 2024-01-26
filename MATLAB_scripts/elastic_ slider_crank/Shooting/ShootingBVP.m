function residual = ShootingBVP(W_X0, Const, Config)

%%                          INITIALIZATION
%   Extract needed quantities
qr = Const.qr;
Q_0 = Const.Q_0;
r_0 = Const.r_0;
r_base = [Const.r_base 0 0]';
time_span = Config.forward_integration_domain;
R_0 = Q2R(Q_0);
A_bar = Const.A_bar;
A = Const.A;

A = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];


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

%%                          Shooting ODEs
%   Define the initial state
y_X0 = [r_X0
        Q_X0
        W_X0];

%   [TODO] Numerical integration of the state vector using the Shooting
%   ODEs
    [~,y] = ode45(@(X,y)CosseratShootingODEs(X,y, Const),time_span,y_X0);
%   [TODO]  Get state at the rod tip
    y_x1 = y(end, :)';
%   [TODO] Get quantities at tip from the state at X=1
r_X1 = zeros(3, 1);  
r_X1 = y_x1(1:3);
Q_X1 = zeros(4, 1);
Q_X1 = y_x1(4:7);

W_X1 = y_x1(8:13);


%%                       BOUNDARY VALUE PROBLEM

%   [TODO]  Compute the wrench balance at the rod tip. In this case, reason
%   about the constrains appling on every element of W_X1.

Rw = A_bar*W_X1;

%% for getting 0 the residual ( static) we need to have the DOF = 0

%   [TODO]  Compute the Geometrical residual. In this section we need to 
%   evaluate the constrains at the rod tip, namely Psi. (the getSE3error
%   function might be used to this aim).
Psi = getSE3error(Q_X1, r_X1, [1 0 0 0]', [0 0 0]');
Psi = A * Psi;
%% The position not allow must be = 0 => Res = 0
%   Compose the residual
residual = [Rw;
            Psi];


end