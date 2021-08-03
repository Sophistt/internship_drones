
function params = crazyfile()
    
    % Params of the crazyfile drone 2.0
    
    Jxx = 1.43 * 10^(-5); % [Kg x m^2] 1.395
    Jyy = 1.43 * 10^(-5); % [Kg x m^2] 1.436
    Jzz = 2.173 * 10^(-5); % [Kg x m^2]
    I = diag([Jxx, Jyy, Jzz]);
    CT  = 3.1582 * 10^(-10);  % [N/rpm^2]
    CD  = 7.9379  * 10^(-12); % [Nm/rpm^2]
    gamma = CD / CT;
    d = 39.73 * 10^(-3); % [m]
    m = 0.033; % [kg]
    g = 9.81;
    
    params.mass = m;   params.grav = g;
    params.Jxx  = Jxx; params.Jyy  = Jyy;  params.Jzz = Jzz;
    params.I    = I;   params.invI = inv(I);
    params.CT   = CT;  params.CD   = CD;   params.gamma = gamma;
    params.arm_length = d;
    
    params.maxangle = 40*pi/180; % you can specify the maximum commanded angle here
    params.maxF     = 2.5*m*g;   % left these untouched from the nano plus
    params.minF     = 0.05*m*g;  % left these untouched from the nano plus
end