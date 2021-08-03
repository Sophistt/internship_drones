
function [x, y] = grid2real(xgrid,ygrid)
    x = (ygrid - 60) ./ 10;
    y = -(xgrid - 30) ./ 10;
    
    x = flip(x); y = flip(y);
end