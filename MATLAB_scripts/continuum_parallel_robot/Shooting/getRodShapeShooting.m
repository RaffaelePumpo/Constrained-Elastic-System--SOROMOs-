function shape = getRodShapeShooting(W_X0, Q_X0, r_X0, Const, Config)
%   Given the wrench at the rod base W_X0, the orientation Q_X0 and the
%   position r_X0, find the corresponding rod shape using the Shooting odes
y_X0 = [r_X0
        Q_X0
        W_X0];

[t,Y] = ode45(@(X,y)CosseratShootingODEs(X, y, Const),Config.forward_integration_domain, y_X0);


%   [TODO] Extract the shape from the ode45 result
shape = Y(:,1:3);
end