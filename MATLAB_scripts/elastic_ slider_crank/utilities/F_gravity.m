function Fg = F_gravity(Q, Const)

R = Q2R(Q);


A   = Const.Area;
g   = Const.g;
rho = Const.rho;

fg = -R'*[0;   
         0; 
         rho*A*g];

Fg = [zeros(3,1);
      fg];


end