function J = getJacobianNumericSBP(chi, Const, Config)


%%                          INITIALIZATION
%   Extract needed quantities
n = length(chi);

%   Define the step of the numerical perturbation
delta = Config.delta;

%   Initialize the Jacobian matrix
J = zeros(n, n);

%%                   COMPUTE JACOBIAN COLUMN-WISE
%   In this section, we want to compute the Jacobian column wise evaluating
%   the differences in the residual vector from a small perturbation of the
%   parameters vector chi. You can use the Forward or Central approach
chi_m = chi - delta*n;
R_m = getResidual(chi_m,Const,Config);
chi_p = chi + delta*n;
R_p = getResidual(chi_p,Const,Config);
J = (R_p-R_m)/delta;

end