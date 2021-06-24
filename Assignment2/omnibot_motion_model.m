function mup = omnibot_motion_model(x,u,dt)
    
    r = 0.25;
    L = 0.3;
    v_x = (r*2/3) * (-u(1)*cos(x(3)) + u(2)*cos(pi/3-x(3)) + u(3)*cos(pi/3+x(3)));
    v_y = (r*2/3) * (u(1)*sin(x(3)) + u(2)*sin(pi/3-x(3)) - u(3)*sin(pi/3+x(3)));
    omega = r/(3*L) * (u(1)+u(2)+u(3));
    mup = x + [ v_x * dt; 
              v_y * dt; 
              omega * dt];
end