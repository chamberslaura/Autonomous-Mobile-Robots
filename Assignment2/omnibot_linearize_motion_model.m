function Gt = omnibot_linearize_motion_model(mup,u)

    x = mup;
    Gt = [ 1 0 (u(1)*sin(x(3)))/60 - (u(2)*sin(x(3) - pi/3))/60 - (u(3)*sin(x(3) + pi/3))/60;
         0 1 (u(1)*cos(x(3)))/60 - (u(2)*cos(x(3) - pi/3))/60 - (u(3)*cos(x(3) + pi/3))/60;
         0  0 1];
end