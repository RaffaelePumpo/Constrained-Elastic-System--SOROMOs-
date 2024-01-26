function [rod1_shape, rod2_shape] = getRobotShapeSBP(chi, Const, Config)


%%                          INITIALIZATION
%   Extract needed quantities
ne = Const.dim_base;
d = Const.d;

%   Decompose variables
q1 = chi(1:ne);
q2 = chi(ne+1:2*ne);


%   [TODO]  Define absolute position and orientation base frame for rod1
Q1_X0 = Const.Q_0;
r1_X0 = Const.r_0;

%   [TODO]  Define absolute position and orientation base frame for rod2
Q2_X0 = Const.Q_0;
r2_X0 = r1_X0 + [d 0 0]';


%   Get rod shape for both limbs
rod1_shape = getRodShapeSBP(q1, Q1_X0, r1_X0, Const, Config);
rod2_shape = getRodShapeSBP(q2, Q2_X0, r2_X0, Const, Config);


end