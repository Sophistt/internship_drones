function [xgrid, ygrid] = real2grid(x, y)
    xgrid = ceil(10 * x)  + 60;
    ygrid = -ceil(10 * y) + 30;
end