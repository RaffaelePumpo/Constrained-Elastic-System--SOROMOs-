function R = ShootingBVP(guess, Const, Config)

%%                          INITIALIZATION
%   Extract needed quantities
d = Const.d;
time_span = Config.forward_integration_domain;

%   Get the initial pose of the first rod. 
%   In this case we take the position of the first joint as reference
Q1_X0 = Const.Q_0;
r1_X0 = Const.r_0;

%   [TODO]  Get the initial pose of the second rod. 
%   In this case we take we should account for the distance between the two
%   sliders
Q2_X0 = Q1_X0;
r2_X0 = r1_X0 + [d 0 0]';


%   [TODO]  Decompose the guess to get the two planar wrenches that are
%   applied at the base of the rods
W1_plane_X0 = guess(1:3);
W2_plane_X0 = guess(4:6);


%   Compose the wrench as the full se(3) wrench
W1_X0 = [0
         0
         W1_plane_X0
         0];
W2_X0 = [0
         0
         W2_plane_X0
         0];


%   [TODO]  Define the initial states for both rods
y1_X0 = [r1_X0
        Q1_X0
        W1_X0];
y2_X0 = [r2_X0
        Q2_X0
        W2_X0];


%   [TODO]  Numerical integration of the CosseratShootingODEs for both
%   the first and second rod.
[~,Y1] = ode45(@(X,y1)CosseratShootingODEs(X,y1, Const),time_span,y1_X0);
[~,Y2] = ode45(@(X,y2)CosseratShootingODEs(X,y2, Const),time_span,y2_X0);

%   [TODO]  Extract the state at the tip
y_x1 = Y1(end, :);
y_x2 = Y2(end, :);


%   [TODO]  Get quantities at tip
r1_X1 = y_x1(1:3)';          
Q1_X1 = y_x1(4:7)';
W1_X1 = y_x1(8:13)';
W1_plane_X1 = W1_X1(3:5);

r2_X1       = y_x2(1:3)';          
Q2_X1       = y_x2(4:7)';
W2_X1 = y_x2(8:13)';
W2_plane_X1 = W2_X1(3:5);


%   [TODO]  Compute the geometrical error, in this case by comparing the
%   position and orientation of the two rod tips.
 Psi = getSE3error(Q2_X1, r2_X1,Q1_X1, r1_X1);
A = [%1 0 0 0 0 0    %   roll
     %0 1 0 0 0 0    %   pitch
     0 0 1 0 0 0    %   yaw
     0 0 0 1 0 0    %   ex
     0 0 0 0 1 0    %   ey
     %0 0 0 0 0 1    %   ez
     ];
A_bar = Const.A_bar;
Psi = A * Psi;
%   [TODO]  Compute balance of wrenches
% Rw = A_bar*(W1_X1+W2_X1);
Rw = W1_plane_X1 + W2_plane_X1;


R = [Psi
     Rw];

end