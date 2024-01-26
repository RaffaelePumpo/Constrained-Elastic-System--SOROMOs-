function J = getJacobianNumericShooting(chi, Const, Config)


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






end