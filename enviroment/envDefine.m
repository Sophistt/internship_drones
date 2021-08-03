
function env = envDefine()
% This function returns the vertices of environment space and obstacles
    space.xmin = -6; space.xmax = 6;
    space.ymin = -3; space.ymax = 3;
    space.zmin = 0;  space.zmax = 2;


    obs.cube_1 = [[.4 .4 .6];[.8 -.2 0]];
    obs.cube_2 = [[.4 .4 .6];[1.8 -0.6 0]];
    obs.cube_3 = [[.4 .4 .6];[2.8 -.2 0]];
    obs.cube_4 = [[.4 .4 .6];[1.8 .2 0]];

    env.space = space;
    env.obs   = obs;
end
