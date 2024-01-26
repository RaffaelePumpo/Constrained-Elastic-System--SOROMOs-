function R = getResidual(chi, Const, Config)

%%                          INITIALIZATION
%   Extract needed quantities
ne = Const.dim_base;
Q_0 = Const.Q_0;
r_0 = Const.r_0;
R_0 = Q2R(Q_0);
qr = Const.qr;
Rz = rotZ(qr);
R_X0 = Rz*R_0;
Q_X0 = R2Q(R_X0);
r_0 = Const.r_0;
r_base = [Const.r_base 0 0]';
r_X0 = r_0-r_base;
%   Decompose variables
q  = chi(1:ne);
A = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];
%%                          STATIC MODEL
%   Call the IGM to get the pose at the tip and generalized forces at the
%   base
[r_X1, Q_X1, Qa_X0] = IGM(chi, Config, Const);


%%                      BOUNDARY VALUE PROBLEM
%   Get the stifness matrix
Kee = Const.Kee;


%   [TODO]  Get the generalized elastic forces
Qee = Kee*q;


%   [TODO]  Compute the residual Rq accounting for the internal balance of
%   the rod
Rq = Qee-Qa_X0;


%   [TODO]  We compute now the geometrical constrains. In this section we 
%   need to evaluate the constrains at the rod tip, namely Psi. (the 
%   getSE3error function might be used to this aim).
Psi = getSE3error(Q_X1, r_X1, [1 0 0 0]', [0 0 0]');
Psi = A * Psi;
%   Compose the residual
R = [Rq;
     Psi];

end