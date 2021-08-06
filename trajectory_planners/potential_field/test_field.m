clc; clear; close all;

des = [0; 0];
Ka  = 0.1;
Kb  = 0.1;
d0  = 0.1;

syms x y;

f = 1/2 * Ka * ((x-des(1))^2 + (y-des(2))^2);
g = -gradient(f, [x, y]);

[X, Y] = meshgrid(-0:.1:6,-3:.1:3);
G1 = subs(g(1),[x y],{X,Y});
G2 = subs(g(2),[x y],{X,Y});
quiver(X,Y,G1,G2)
hold on;

q = [4.4; -1.9];
for i = 1:20
    dx = - Ka * (q(1)-des(1));
    dy = - Ka * (q(2)-des(2));
    q(1) = q(1) + dx; 
    q(2) = q(2) + dy;
    plot(q(1), q(2), 'or');
end

%%

% x = -5:0.2:5;
% y = x';
% f = 1/2 * Ka * ((x-des(1)).^2 + (y-des(2)).^2);
% 
obs.pos = [3;3];
obs.shape = [1;1];
% 
% dq = disToObs([x,y], obs);
% 
% Uobs = 1/2 * Kb * (1/dq - 1/d0);
% 
% figure;
% surf(x,y,f)