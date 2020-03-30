% orbit([-5; 0])

% plotOrbit()

p_veh = [2 1 1]';
p_obj = [1 1 1]';
p_goal= [1 2 1]';

dp = (p_goal - p_veh);
th = wrapToPi(atan2(dp(2),dp(1)) - pi/2);

R_ov = [cos(th) -sin(th); sin(th) cos(th)];
R_ov*p_goal(1:2);
t_ov = p_veh(1:2);
T_ov = [R_ov t_ov;0 0 1];

ptt = p_obj
T_vo = [R_ov' -R_ov'*t_ov; 0 0 1];
T_vo * ptt
a=5;


function g = orbit(q)
    q_c = [0; 0];
    q_h = q-q_c;
    k = 1;
    r = 8;
    w = 2;
    scale = 1;
    gam = k*(r^2 - q_h'*q_h);
    A = [gam w; -w gam];
    g = A*q_h;
    g = g/norm(g)*scale;
end


function plotOrbit()
    check_state =[0; 0];
    in_state = check_state;

    cushion = 10;
    spacing = 1;
    x_in = in_state(1)-cushion:spacing:in_state(1)+cushion;  
    y_in = in_state(2)-cushion:spacing:in_state(2)+cushion;
    [x,y] = meshgrid(x_in, y_in);
    xVec = zeros(size(x));
    yVec = zeros(size(y));
    for i=1:length(x_in)
        for j=1:length(y_in)
            check_state(2) = x_in(i);
            check_state(1) = y_in(j);
            velo = orbit(check_state);
            xVec(i,j) = velo(1);
            yVec(i,j) = velo(2);
        end
    end
    quiver(x,y,xVec,yVec,'k-')
end