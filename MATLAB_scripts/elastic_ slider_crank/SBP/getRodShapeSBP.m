function shape = getRodShapeSBP(q, Const, Config)
%   Given the state vector chi, find the corresponding rod shape using the
%   Forward Kinematics
Q_0 = Const.Q_0;
R_0 = Q2R(Q_0);
qr = Const.qr;
Rz = rotZ(qr);
R_X0 = Rz*R_0;
Q_X0 = R2Q(R_X0);
r_0 = Const.r_0;
r_base = [Const.r_base 0 0]';
r_X0 = r_0-r_base;
y_X0 = [Q_X0;r_X0];
[~, Y] = ode45(@(X,y)ForwardKinematics(X,y,q,Const),Config.forward_integration_domain, y_X0);
shape = Y(:,5:7);
end