function shape = getRodShapeShooting(W_X0, Const, Config)
%   Given the wrench at the rod base W_X0, find the corresponding rod 
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


    [t,Y] = ode45(@(X,y)CosseratShootingODEs(X, y, Const),Config.forward_integration_domain, y_X0);
%   [TODO] Extract the shape from the ode45 result
shape = Y(:,1:3);

end