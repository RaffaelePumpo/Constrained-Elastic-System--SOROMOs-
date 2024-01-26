function Const = defineCosseratRod(Config)



%%  GEOMETRICAL PARAMETERS

%   Lenght of the rod
Const.L = 0.382;

%   Radius Cross section
Const.Rc  = 0.001;
Const.r_base = 0.03;

%   Area
A = pi*Const.Rc^2;
Const.Area = A;

%   Geometrical moment of inertia
Const.J      = zeros(3,3);
Const.J(1,1) = pi*Const.Rc^4/2;
Const.J(2,2) = pi*Const.Rc^4/4;
Const.J(3,3) = pi*Const.Rc^4/4;



%%  MATERIAL PARAMETERS
%   Specific weight
Const.rho = 7800;

%   Shear modulus
Const.G = 80e9;

%   Young modulus
Const.E  = 210e9;

%   Internal damping
Const.mu = 1e-4;



%%  STIFFNESS AND INERTIA OF CROSS SECTION

Const.GIxx = Const.G*Const.J(1,1);
Const.EIyy = Const.E*Const.J(2,2);
Const.EIzz = Const.E*Const.J(3,3);
Const.EA = Const.E*Const.Area;
Const.GA = Const.G*Const.Area;

Const.GI = Const.GIxx;
Const.EI = Const.EIyy;




%%  STRAIN BASED PARAMETERIZATION


% Definition of actuated values
% K1, K2, K3, ...
% if value is 1 -> the variable is actuated
% if value is 0 -> the variable is not actuated
Const.V_a = [0, 0, 1, 0, 0, 0];

Const.dim_V_a = sum(Const.V_a);

%   Define the size of the parameterisation
ne = 3;
Const.dim_base_k = ne*Const.V_a;
%   Compute size of q
Const.dim_base   = Const.V_a*Const.dim_base_k';


%   Automatically define matrix B
M_selec = eye(6,6);
[~,col] = find(Const.V_a==1);
Const.B = M_selec(:,col);

%   Define constant strain
Const.Xi_c = [0;0;0;1;0;0];

Const.v0 = Const.Xi_c(4:6);



%%  STIFFNESS AND DAMPING MATRICES

H = diag([Const.GI,Const.EI,Const.EI,Const.EA,Const.GA,Const.GA]);
Const.H = H;

Const.Kse = H(4:6, 4:6);
Const.Kbt = H(1:3, 1:3);

[Kee, ~] = computeGeneralisedStiffnessDampingMatrices(Const, Config);

Const.Kee = Kee;


%%  GEOMETRICAL CONSTRAINS
I = eye(6);

%   Joint DoFs
joint_dofs = [0 0 0 1 0 0]';
range = (1:6)';

%   Account for the plane
P = I(3:5, :);

range = P*range;

index_row = 1;
for it=range(1):range(end)

    if(joint_dofs(it) == 0)
        Aj(index_row, :) = I(it, :);
        index_row = index_row + 1;       
    end
end

Const.A = Aj;



index_row = 1;
for it=range(1):range(end)

    if(joint_dofs(it) == 1)
        A_bar(index_row, :) = I(it, :);
        index_row = index_row + 1;       
    end
end

Const.A_bar = A_bar;

end