function EI = Root_EI(Const,Config,real_data)
result_shooting = Shooting(Const,Config);
xv = Const.EI;
dx = 15;
R = result_shooting(end,1)-real_data;
epsilon = 1e-6;
i = 0;

% Iterative root-finding process
while (norm(R) > epsilon && i < 1000)
        Const.EI = xv + dx;
        result_shooting = Shooting(Const,Config);
        result_shooting(end,1)
        Rd = result_shooting(end,1)-real_data;
        J = (Rd - R) / dx;


    % Update root estimate
    xup = -pinv(J) * R;
    Const.EI = xv + xup;
    result_shooting = Shooting(Const,Config);
    R = result_shooting(end,1)-real_data;
    i = i + 1;
end

EI = Const.EI;
end