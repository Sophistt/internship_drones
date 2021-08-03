clc; clear; close all;

env = envDefine();

% Define grid map
MAP = int8(zeros(10*(env.space.ymax-env.space.ymin), ...
                 10*(env.space.xmax-env.space.xmin)));
% Define goal
GoalRegister= int8(zeros(10*(env.space.ymax-env.space.ymin), ...
                         10*(env.space.xmax-env.space.xmin)));
[xGgrid, yGgrid] = real2grid(5,0);
GoalRegister(yGgrid,xGgrid)=1;

% Set obstacles in Grid map
obs = fieldnames(env.obs);
for i=1:length(obs)
    k = obs(i);
    key = k{1};
    value = env.obs.(key);
    
    xmin = value(2,1); ymin = value(2,2);
    xmax = xmin+value(1,1); ymax = ymin+value(1,2);
    [xgmin, ygmin] = real2grid(xmin, ymin);
    [xgmax, ygmax] = real2grid(xmax, ymax);
    MAP(ygmax:ygmin,xgmin:xgmax)=1;
end

% Set initial point
[xSgrid, ySgrid] = real2grid(0,0);

Connecting_Distance=8;
OptimalPath = ASTARPATH(xSgrid,ySgrid,MAP,GoalRegister,Connecting_Distance);

% Transform to real coordinates from grid
[x, y] = grid2real(OptimalPath(:,1), OptimalPath(:,2));
xx = x(1):.1:x(end);
yy = spline(x, y, xx);


imagesc((MAP))
    colormap(flipud(gray));

hold on
plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
plot(OptimalPath(:,2),OptimalPath(:,1),'r')
legend('Goal','Start','Path')


figure;
plot(x,y,'o',xx,yy);
xlim([-6, 6]);
ylim([-3, 3]);
