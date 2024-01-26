function [rod1_shape, rod2_shape] = getRobotShapeShooting(guess, Const, Config)


%%                          INITIALIZATION
%   Extract needed quantities
d = Const.d;

%   Get the initial pose of the first rod. 
%   In this case we take the position of the first joint as reference
Q1_X0 = Const.Q_0;
r1_X0 = Const.r_0;

%   [TODO]  Get the initial pose of the second rod. 
%   In this case we take we should account for the distance between the two
%   sliders
Q2_X0 = Q1_X0;
r2_X0 = r1_X0 + [d 0 0]';


%   [TODO]  Decompose the guess to get the two planar wrenches that are
%   applied at the base of the rods
W1_plane_X0 = guess(1:3);
W2_plane_X0 = guess(4:6);


%   Compose the wrench as the full se(3) wrench
W1_X0 = [0
         0
         W1_plane_X0
         0];
W2_X0 = [0
         0
         W2_plane_X0
         0];

%   [TODO]  Numerical integration of the CosseratShootingODEs for both
%   the first and second rod.
rod1_shape = getRodShapeShooting(W1_X0, Q1_X0, r1_X0, Const, Config);
rod2_shape = getRodShapeShooting(W2_X0, Q2_X0, r2_X0, Const, Config);


end