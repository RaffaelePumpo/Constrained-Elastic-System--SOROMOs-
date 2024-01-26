function Config = simulationConfigurations()



%   Properties for Cosserat ODEs integration
Config.dX = 10^-3;
Config.forward_integration_domain = 0:Config.dX:1;
Config.backward_integration_domain = 1:-Config.dX:0;


%   Threshold residual norm
Config.r_min = 1*10^-7;


%   Value of the numerical perturbation
Config.delta = 1e-6;

end