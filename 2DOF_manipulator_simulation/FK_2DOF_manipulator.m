function u = FK_2DOF_manipulator(th)
    global l1
    global l2
    th1 = th(1);
    th2 = th(2);
    x = l1*cos(th1) + l2*cos(th1+th2);
    y = l1*sin(th1) + l2*sin(th1+th2);
    u = [x y];
end