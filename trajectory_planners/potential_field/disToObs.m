
function [dis, dx, dy] = disToObs(p, obs)
    dx = max(abs(p(1)-obs.pos(1))- obs.shape(1)./2, 0.001);
    dy = max(abs(p(2)-obs.pos(2))- obs.shape(2)./2, 0.001);
    
    dis = sqrt(dx*dx+dy*dy);
end