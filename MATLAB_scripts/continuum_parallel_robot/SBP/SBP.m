function [rod1_shape, rod2_shape] = SBP(Const, Config)


%%                          INITIALIZATION
%   Extract needed quantities
ne = Const.dim_base;

%   Initialize the components of the state vector
q1 = zeros(ne, 1);
q2 = zeros(ne, 1);
W_X1_plane = zeros(3, 1);

%   Use some meaningfull initial values
q1(1) = -1;
q2(1) = 1;

%   Initialize the state vector chi
chi = [q1
       q2
       W_X1_plane
       ];



%%                          FIND SOLUTION
%   In this section, find the solution of the problem with either fsolve
%   or a Newton-Raphson method

%   Compute residual
chi = fsolve(@(chi)getResidual(chi,Const,Config),chi);


%%                      GET THE ROBOT SHAPE
[rod1_shape, rod2_shape] = getRobotShapeSBP(chi, Const, Config);
   


end