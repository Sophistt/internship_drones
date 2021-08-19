clc; clear; close all;

x = -0:0.03:4;
y = -2:0.03:2;

des = [3; 0];
obs.pos = [1;1];
obs.shape = [1;1];

Ka  = 0.01;
Kb  = 0.000001;
d0  = 0.2;

[X,Y] = meshgrid(x, y);

dis = zeros(length(x), length(y));
Uobs = zeros(length(x), length(y));
for i = 1:length(x)
    for j = 1:length(y)
        [dis(j,i), ~, ~] = disToObs([x(i),y(j)], obs);
        if dis(j, i) < d0
            Uobs(j,i) = 1/2 * Kb *(1/dis(j,i)-1/d0)^2;
        else
            Uobs(j,i) = 0;
        end
    end
end


Z = 1/2 * Ka * ((X-des(1)).^2 + (Y-des(2)).^2);
Z = Z + Uobs;
surf(X,Y,Z)
hold on;


p(1) = 0;
q(1) = 2;
r(1) = 1/2 * Ka * ((p(1)-des(1)).^2 + (q(1)-des(2)).^2); 
t_step = 0.01;
for t = 1:30000
    Fatt_x = - Ka * (p(t)-des(1));
    Fatt_y = - Ka * (q(t)-des(2));
    [dis, dx, dy]    = disToObs([p(t); q(t)], obs);
    if dis < d0
        Frep_x =   Kb * (1/dis-1/d0)*(1/dis)^2*((p(t)-dx)/dis);
        Frep_y =   Kb * (1/dis-1/d0)*(1/dis)^2*((q(t)-dy)/dis);
    else
        Frep_x = 0;
        Frep_y = 0;
    end
    
    
    p(t+1) = p(t) + t_step * (Fatt_x + 0.3 * Frep_x);
    q(t+1) = q(t) + t_step * (Fatt_y + 0.3 * Frep_y);
    r(t+1) = 1/2 * Ka * ((p(t+1)-des(1)).^2 + (q(t+1)-des(2)).^2); 
    
end
plot3(p,q,r, 'or', 'LineWidth', 2);
