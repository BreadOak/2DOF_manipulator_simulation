function th_2dot = FD_2DOF_manipulator(Torque)
    global l1
    global l2
    global m1
    global m2
    global g
    
    th1 = th(1);
    th2 = th(2);
    th_dot1 = th_dot(1);
    th_dot2 = th_dot(2);
    M11 = (1/3)*l2^2*m2 + l1*l2*m2*cos(th2) + l1^2*((1/3)*m1+m2);
    M12 = (1/3)*l2^2*m2 + (1/2)*l1*l2*m2*cos(th2);
    M21 = M12;
    M22 = (1/3)*l2^2*m2;
    M = [ M11 M12;
          M21 M22];
    V = [ -(1/2)*m2*l1*l2*sin(th2)*th_dot2^2 - m2*l1*l2*sin(th2)*th_dot1*th_dot2;
                                               (1/2)*m2*l1*l2*sin(th2)*th_dot1^2];
    G = [ (1/2)*m2*l2*g*cos(th1+th2) + ((1/2)*m1+m2)*l1*g*cos(th1);
                                       (1/2)*m2*l2*g*cos*(th1+th2)];
    th_2dot = inv(M)*(Torque - V - G);
end