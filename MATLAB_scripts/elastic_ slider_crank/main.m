close all
clear all
clc


addpath("ODEs/")
addpath("utilities/")
addpath("rod_properties/")
addpath("SBP/")
addpath("Shooting/")


%   Configurations for the simulation
Config = simulationConfigurations();

%   Define properties for the rod
Const = defineCosseratRod(Config);

%   Value of gravity
Const.g = 0;

%   Force at the rod tip
Const.weight = 0.1;

%   The angle of rotation (in radians)
Const.qr = deg2rad(90);

%   Position of the rod base, defining the position of the rod base
%   with respect to the frame it is attached to.
Const.r_0 = [0;0;0];

%   Quaternion of rod base [w x y z] defining the orientation of the rod
%   base with respect to the frame it is attached to. 
Const.Q_0 = [1, 0, 0, 0]';

%   orientation (Q_0) and position (r_0) defines how the rod base is
%   attached to the joint body
real_data = 0.302;
%EI = Root_EI(Const,Config,real_data);
result_sbp = SBP(Const, Config);
result_shooting = Shooting(Const,Config);




plot(result_shooting(:,1), result_shooting(:,2),'-b','LineWidth',2, 'DisplayName', 'Shooting')
hold on
plot(result_sbp(:,1), result_sbp(:,2),'--r','LineWidth',2, 'DisplayName', 'SBP')
grid on
axis equal


grid on
hold off
axis equal

 %xlim([-0.1*Const.L 1.1*Const.L])
 %ylim([-0.6*Const.L 0.6*Const.L])
legend('Location', 'northwest')

drawnow




