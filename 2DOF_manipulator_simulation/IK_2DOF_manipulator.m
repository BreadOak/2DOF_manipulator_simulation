function [th,th_dot,th_2dot] = IK_2DOF_manipulator(u,u_dot,u_2dot)
    global l1
    global l2

    x = u(1);
    y = u(2);
    c2 = (x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2);
    s2 = sqrt(1-c2*c2);
    th2 = atan2(s2,c2);
   
    c1 = det([x -l2*sin(th2); y l1+l2*cos(th2)])/det([l1+l2*cos(th2) -l2*sin(th2); l2*sin(th2) l1+l2*cos(th2)]);
    s1 = det([l1+l2*cos(th2) x; l2*sin(th2) y])/det([l1+l2*cos(th2) -l2*sin(th2); l2*sin(th2) l1+l2*cos(th2)]);
    th1 = atan2(s1,c1);
    th = [th1 th2];
   
    G = [ -(l1*sin(th1)+l2*sin(th1+th2)) -(l2*sin(th1+th2));
          (l1*cos(th1)+l2*cos(th1+th2))  (l2*cos(th1+th2))];
    th_dot = inv(G)*u_dot;
   
    H = G*th_dot;
   
    th_2dot = inv(G)*(u_2dot - th_dot.' * H * th_dot);
end