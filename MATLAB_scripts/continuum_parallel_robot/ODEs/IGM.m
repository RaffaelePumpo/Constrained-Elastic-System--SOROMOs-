function [r_X1, Q_X1, Qa_X0] = IGM(q, Wplane, Q_X0, r_X0, Config, Const)

%   Extract needed quantities
ne = Const.dim_base;

%   Initialization
forward_y0 = [Q_X0;
              r_X0];

%   Forward integration
[~, Y] = ode45(@(X, y) ForwardKinematics(X, y, q, Const), ...
                    Config.forward_integration_domain, forward_y0);

%   Extract state at X=1
y_tip = Y(end,:);

Q_X1 = y_tip(1:4)';
r_X1 = y_tip(5:7)';
R_X1 = Q2R(Q_X1);


%   Force at the tip
F1 = zeros(6, 1);
F1(3:5) = Wplane;


projection_matrix = [  R_X1  , zeros(3);
                     zeros(3),   R_X1  ];

%   Project wrench into local coordinates
Lambda_X1 = projection_matrix'*F1;

%   Initialization
backward_y1 = [ Q_X1;
                r_X1;
                Lambda_X1;
                zeros(ne, 1)];


%   Backward integration
[~, Y] = ode45(@(X, y) BackwardStatics(X, y, q, Const), ...
                    Config.backward_integration_domain, backward_y1);

%   Extract state at X=0
y_base = Y(end,:);

Lambda_X0 = y_base(8:13)';
Qa_X0 = y_base(14:end)';


end

