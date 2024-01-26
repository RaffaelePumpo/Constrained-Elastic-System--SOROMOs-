function R = getResidual(chi, Const, Config)
%   Definition of the state chi
%   q1
%   q2
%   mz1_X1
%   fx1_X1
%   fy1_X1
%   mz2_X1
%   fx2_X1
%   fy2_X1


%   Extract needed quantities
ne = Const.dim_base;
d = Const.d;

%   Decompose variables
q1 = chi(1:ne);
q2 = chi(ne+1:2*ne);

index = 2*ne + 1;
Wplane1 = chi(index:index+2);
index = index + 3;
Wplane2 = chi(index:end);


%   Now we need to define the stqrting point of the rod 1
Q1_X0 = Const.Q_0;
r1_X0 = Const.r_0;

%   For the rod 2, we know it shifted of d along x
Q2_X0 = Const.Q_0;
r2_X0 = r1_X0 + [d 0 0]';

%   Call the IGM to get the pose at the tip and generalized forces at the
%   base
[r1_X1, Q1_X1, Qa1_X0] = IGM(q1, Wplane1, Q1_X0, r1_X0, Config, Const);

[r2_X1, Q2_X1, Qa2_X0] = IGM(q2, Wplane2, Q2_X0, r2_X0, Config, Const);


%   Get the stifness matrix
Kee = Const.Kee;


%   Get the generalized elastic forces
Qe1 = Kee*q1;
Qe2 = Kee*q2;


%   Compare the generalized forces
Rq1 = Qe1 - Qa1_X0;
Rq2 = Qe2 - Qa2_X0;


%   Geometrical constrains
Psi = getPsi(Q2_X1, r2_X1, Q1_X1, r1_X1);

A = [%1 0 0 0 0 0    %   roll
     %0 1 0 0 0 0    %   pitch
     0 0 1 0 0 0    %   yaw
     0 0 0 1 0 0    %   ex
     0 0 0 0 1 0    %   ey
     %0 0 0 0 0 1    %   ez
     ];
Psi = A*Psi;

%   Wrench balance
Rw = Wplane1 + Wplane2;

%   Compose the residual
R = [
     Rq1
     Rq2
     Rw
     Psi
     ];

end