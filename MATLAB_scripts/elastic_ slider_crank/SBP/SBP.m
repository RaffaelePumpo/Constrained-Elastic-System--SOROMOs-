function shape = SBP(Const, Config)


%%                          INITIALIZATION
%   Extract needed quantities
ne = Const.dim_base;
q = zeros(ne, 1);

%   [TODO]  Initialize the state vector chi. This vector should contain the
%   state vector for the elastic slider-crank mechanism.
n_y0 = 0;
m_z0 = 0;
chi = [q;n_y0;m_z0];


%%                          FIND SOLUTION
%   In this section, find the solution of the problem with either fsolve
%   or a Newton-Raphson method

%   Compute residual
chi = fsolve(@(chi)getResidual(chi,Const,Config),chi);

q = chi(1:ne);

%%                          GET THE ROD SHAPE
shape = getRodShapeSBP(q, Const, Config);
   


end