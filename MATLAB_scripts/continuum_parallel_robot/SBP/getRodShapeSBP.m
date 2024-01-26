function shape = getRodShapeSBP(q, Q_X0, r_X0, Const, Config)
%   Given the state vector chi, find the corresponding rod shape using the
%   Forward Kinematics
y_X0 = [Q_X0;r_X0];

[~, Y] = ode45(@(X,y)ForwardKinematics(X,y,q,Const),Config.forward_integration_domain, y_X0);

shape = Y(:,5:7);
end