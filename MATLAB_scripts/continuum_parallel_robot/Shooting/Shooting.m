function [rod1_shape, rod2_shape] = Shooting(Const, Config)

%%                          INITIALIZATION
%   Initialize the state vector
W = zeros(6, 1);

%%                          FIND SOLUTION
%   In this section, find the solution of the problem with either fsolve
%   and a Newton-Raphson method

%   Compute residual
W = fsolve(@(W)ShootingBVP(W, Const, Config),W);

%%   Now get the shape of the robot
[rod1_shape, rod2_shape] = getRobotShapeShooting(W, Const, Config);

end