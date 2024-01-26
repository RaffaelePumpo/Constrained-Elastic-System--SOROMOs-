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

%   The distance between the two joints
Const.d = 0.38;

%   Position of the rod base
Const.r_0 = [0;0;0];

%   Quaternion of rod base [w x y z]
Const.Q_0 = R2Q(rotZ(pi/2));





[rod1_shape_sbp, rod2_shape_sbp] = SBP(Const, Config);
[rod1_shape_sht, rod2_shape_sht] = Shooting(Const,Config);

% 
plot(rod1_shape_sht(:,1), rod1_shape_sht(:,2),'-b','LineWidth',2, 'DisplayName', 'Shooting')
hold on
plot(rod2_shape_sht(:,1), rod2_shape_sht(:,2),'-b','LineWidth',2, 'HandleVisibility', 'off')

plot(rod1_shape_sbp(:,1), rod1_shape_sbp(:,2),'--r','LineWidth',2, 'DisplayName', 'SBP')
hold on
plot(rod2_shape_sbp(:,1), rod2_shape_sbp(:,2),'--r','LineWidth',2, 'HandleVisibility', 'off')
grid on
axis equal


grid on
hold off
axis equal

% xlim([-0.1*Const.L 1.1*Const.L])
% ylim([-0.6*Const.L 0.6*Const.L])
legend('Location', 'northwest')

drawnow




