global l1 l2 m1 m2 g h

l1 = 0.5; % m
l2 = 0.5; % m
m1 = 1;   % kg
m2 = 1;   % kg
g = 9.81; % m/s^2
h = 0.01; % second(sampling time)

a = 0;
b = 0;

r = 0.2; % m
Tf = 2;  % sec

% Make empty desired list
x_des_list = [];
y_des_list = [];

x_dot_des_list = [];
y_dot_des_list = [];

x_2dot_des_list = [];
y_2dot_des_list = [];

th1_des_list = [];
th2_des_list = [];

th1_dot_des_list = [];
th2_dot_des_list = [];

th1_2dot_des_list = [];
th2_2dot_des_list = [];

Torque1_list = [];
Torque2_list = [];

% Make empty act list
th1_2dot_act_list = [];
th2_2dot_act_list = [];

th1_dot_act_list = [];
th2_dot_act_list = [];

th1_act_list = [];
th2_act_list = [];

x_2dot_act_list = [];
y_2dot_act_list = [];

x_dot_act_list = [];
y_dot_act_list = [];

x_act_list = [];
y_act_list = [];

[traj_th, traj_th_dot, traj_th_2dot] = JointTrajectory(0, 2*pi, Tf, h);

for i = 1 : length(traj_th)
    
    % theta trajectory
    th = traj_th(i);
    th_dot = traj_th_dot(i);
    th_2dot = traj_th_2dot(i);
    
    % make circle trajectory
    x = r*sin(th) + a;
    y = r*cos(th) + b;
    
    x_des_list(end+1) = x; 
    y_des_list(end+1) = y; 
    
    x_dot =  r*cos(th) * th_dot;
    y_dot = -r*sin(th) * th_dot;
    
    x_dot_des_list(end+1) = x_dot;
    y_dot_des_list(end+1) = y_dot;
    
    x_2dot = -r*sin(th) * th_dot^2 + r*cos(th) * th_2dot;
    y_2dot = -r*cos(th) * th_dot^2 - r*sin(th) * th_2dot;
    
    x_2dot_des_list(end+1) = x_2dot;
    y_2dot_des_list(end+1) = y_2dot;
    
    u_des = [x ; y];
    u_dot_des = [x_dot ; y_dot];
    u_2dot_des = [x_2dot ; y_2dot];

    % Inverse Kinematics
    [th_des, th_dot_des, th_2dot_des] = IK_2DOF_manipulator(u_des, u_dot_des, u_2dot_des);
    th1_des_list(end+1) = th_des(1);
    th2_des_list(end+1) = th_des(2);
    
    th1_dot_des_list(end+1) = th_dot_des(1);
    th2_dot_des_list(end+1) = th_dot_des(2);
    
    th1_2dot_des_list(end+1) = th_2dot_des(1);
    th2_2dot_des_list(end+1) = th_2dot_des(2);
    
    % Inverse Dynamics
    Torque = ID_2DOF_manipulator(th_des,th_dot_des,th_2dot_des);
    Torque1_list(end+1) = Torque(1);
    Torque2_list(end+1) = Torque(2);
    
    % Forward Dynamics & RK4th method
    [th_act, th_dot_act, th_2dot_act] = FD_2DOF_manipulator(Torque, th_des, th_dot_des);
    th1_act_list(end+1) = th_act(1);
    th2_act_list(end+1) = th_act(2);
    
    th1_dot_act_list(end+1) = th_dot_act(1);
    th2_dot_act_list(end+1) = th_dot_act(2);
    
    th1_2dot_act_list(end+1) = th_2dot_act(1);
    th2_2dot_act_list(end+1) = th_2dot_act(2);
    
    % Forward Kinematics
    [u_act, u_dot_act, u_2dot_act] = FK_2DOF_manipulator(th_act, th_dot_act, th_2dot_act);
    x_act_list(end+1) = u_act(1);
    y_act_list(end+1) = u_act(2);

end

draw_2DOF_manipulator(th1_act_list, th2_act_list);

% plot desired values about X, Y
figure;
subplot(3,2,1)
plot(0:h:Tf-h, x_des_list)
title('Desired Trajectory X')
subplot(3,2,2)
plot(0:h:Tf-h, y_des_list)
title('Desired Trajectory Y')
subplot(3,2,3)
plot(0:h:Tf-h, x_dot_des_list)
title('Desired Trajectory X dot')
subplot(3,2,4)
plot(0:h:Tf-h, y_dot_des_list)
title('Desired Trajectory Y dot')
subplot(3,2,5)
plot(0:h:Tf-h, x_2dot_des_list)
title('Desired Trajectory X 2dot')
subplot(3,2,6)
plot(0:h:Tf-h, y_2dot_des_list)
title('Desired Trajectory Y 2dot')

% plot desired values about Theta1, Theta2
figure;
subplot(3,2,1)
plot(0:h:Tf-h, th1_des_list)
title('Desired Trajectory Theta1')
subplot(3,2,2)
plot(0:h:Tf-h, th2_des_list)
title('Desired Trajectory Theta2')
subplot(3,2,3)
plot(0:h:Tf-h, th1_dot_des_list)
title('Desired Trajectory Theta1 dot')
subplot(3,2,4)
plot(0:h:Tf-h, th2_dot_des_list)
title('Desired Trajectory Theta2 dot')
subplot(3,2,5)
plot(0:h:Tf-h, th1_2dot_des_list)
title('Desired Trajectory Theta1 2dot')
subplot(3,2,6)
plot(0:h:Tf-h, th2_2dot_des_list)
title('Desired Trajectory Theta2 2dot')

% plot Torque
figure;
subplot(2,1,1)
plot(0:h:Tf-h, Torque1_list)
title('Torque 1')
subplot(2,1,2)
plot(0:h:Tf-h, Torque2_list)
title('Torque 2')

% plot actual values & error about Theta1, Theta1
figure;
subplot(6,2,1)
plot(0:h:Tf-h, th1_act_list)
title('Actual Trajectory Theta1')
subplot(6,2,2)
plot(0:h:Tf-h, th1_des_list - th1_act_list)
title('error')
subplot(4,2,3)
plot(0:h:Tf-h, th2_act_list)
title('Actual Trajectory Theta2')
subplot(4,2,4)
plot(0:h:Tf-h, th2_des_list - th2_act_list)
title('error')
subplot(4,2,5)
plot(0:h:Tf-h, th1_dot_act_list)
title('Actual Trajectory Theta1 dot')
subplot(4,2,6)
plot(0:h:Tf-h, th2_dot_act_list)
title('Actual Trajectory Theta2 dot')
subplot(4,2,7)
plot(0:h:Tf-h, th1_2dot_act_list)
title('Actual Trajectory Theta1 2dot')
subplot(4,2,8)
plot(0:h:Tf-h, th2_2dot_act_list)
title('Actual Trajectory Theta2 2dot')

% plot actual values & error about X, Y
figure;
subplot(3,2,1)
plot(0:h:Tf-h, x_act_list)
title('Actual Trajectory Theta1')
subplot(3,2,2)
plot(0:h:Tf-h, x_des_list - x_act_list)
title('error')
subplot(3,2,3)
plot(0:h:Tf-h, y_act_list)
title('Actual Trajectory Theta2')
subplot(3,2,4)
plot(0:h:Tf-h, y_des_list - y_act_list)
title('error')

% compare desired output with actual output 
figure;
subplot(1,3,1)
plot(x_des_list, y_des_list)
title('Desired Output')
subplot(1,3,2)
plot(x_act_list, y_act_list)
title('Actual Output')
subplot(1,3,3)
plot(x_des_list - x_act_list, y_des_list - y_act_list)
title('error')

function draw_2DOF_manipulator(th1_list, th2_list)

    global l1 
    global l2 
    close all;
    figure;
    
    for i = 1:1:length(th1_list)
        th1 = th1_list(i);
        th2 = th2_list(i);

        p0 = [0,0,1]';
        p1 = [l1*cos(th1),l1*sin(th1),1]';
        p2 = [l1*cos(th1)+l2*cos(th1+th2),l1*sin(th1)+l2*sin(th1+th2),1]';
        plot(p0(1),p0(2))
        hold on;
        grid on
        pbaspect([1 1 1])
        line([0 0],[-2 2])
        line([-2 2],[0 0])

        line([p0(1) p1(1) p2(1)],[p0(2) p1(2) p2(2)])
        plot(p0(1),p0(2),'ro')
        plot(p1(1),p1(2),'ro')
        plot(p2(1),p2(2),'ro')
        hold off;
        drawnow;
    end
    
end

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
    
    th = [th1 ; th2];
   
    G1 = [ -(l1*sin(th1)+l2*sin(th1+th2)) -(l2*sin(th1+th2))];
    G2 = [  (l1*cos(th1)+l2*cos(th1+th2))  (l2*cos(th1+th2))];
    
    G = [G1 ; G2];
      
    th_dot = inv(G)*u_dot;
    
    H1 = [-(l1*cos(th1)+l2*cos(th1+th2)) -(l2*cos(th1+th2));
                      -(l2*cos(th1+th2)) -(l2*cos(th1+th2))];
                    
    H2 = [-(l1*sin(th1)+l2*sin(th1+th2)) -(l2*sin(th1+th2));
                      -(l2*sin(th1+th2)) -(l2*sin(th1+th2))];
   
    th_2dot = inv(G) * (u_2dot - [th_dot.'*H1*th_dot ; th_dot.'*H2*th_dot]);

end

function Torque = ID_2DOF_manipulator(th,th_dot,th_2dot)

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
                                        (1/2)*m2*l2*g*cos(th1+th2)];
    Torque = M * th_2dot + V + G;
    
end

function [u, u_dot, u_2dot] = FK_2DOF_manipulator(th,th_dot,th_2dot)

    global l1
    global l2
    
    th1 = th(1);
    th2 = th(2);
    x = l1*cos(th1) + l2*cos(th1+th2);
    y = l1*sin(th1) + l2*sin(th1+th2);
    u = [x y];
    
    
    G1 = [ -(l1*sin(th1)+l2*sin(th1+th2)) -(l2*sin(th1+th2))];
    G2 = [  (l1*cos(th1)+l2*cos(th1+th2))  (l2*cos(th1+th2))];
    
    G = [G1 ; G2];
    
    u_dot = G * th_dot;
    
    H1 = [-(l1*cos(th1)+l2*cos(th1+th2)) -(l2*cos(th1+th2));
                      -(l2*cos(th1+th2)) -(l2*cos(th1+th2))];
                    
    H2 = [-(l1*sin(th1)+l2*sin(th1+th2)) -(l2*sin(th1+th2));
                      -(l2*sin(th1+th2)) -(l2*sin(th1+th2))];
 
    u_2dot = G * th_2dot + [th_dot.' * H1 * th_dot ; th_dot.' * H2 * th_dot];

end

function [th, th_dot, th_2dot] = FD_2DOF_manipulator(Torque, th, th_dot)

    global l1
    global l2
    global m1
    global m2
    global g
    global h
    
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
                                        (1/2)*m2*l2*g*cos(th1+th2)];
                                    
    th_2dot = inv(M)*(Torque - V - G);
    
    func1 = @(th, th_dot) th_dot;
    func2 = @(th, th_dot) inv(M)*(Torque - V - G);
    
    k11 = func1(th, th_dot);
    k21 = func2(th, th_dot);
    
    k12 = func1(th + (h*k11)/2, th_dot + (h*k21)/2);
    k22 = func2(th + (h*k11)/2, th_dot + (h*k21)/2);
    
    k13 = func1(th + (h*k12)/2, th_dot + (h*k22)/2);
    k23 = func2(th + (h*k12)/2, th_dot + (h*k22)/2);
    
    k14 = func1(th + (h*k13), th_dot + (h*k23));
    k24 = func2(th + (h*k13), th_dot + (h*k23));
    
    d_th     = h*(k11 + 2*k12 + 2*k13 + k14)/6;
    d_th_dot = h*(k21 + 2*k22 + 2*k23 + k24)/6;
    
    th     = th     + d_th;
    th_dot = th_dot + d_th_dot;
    
end

function [traj,dtraj,ddtraj] = JointTrajectory(thetastart, thetaend, Tf, s_time)

    N = Tf/s_time;
    timegap = Tf / (N - 1);
    traj = zeros(size(thetastart, 1), N);
    dtraj = zeros(size(0, 1), N);
    ddtraj = zeros(size(0, 1), N);

    for i = 1: N
         [s, ds, dds] = CubicTimeScaling(Tf, timegap * (i - 1));
         
         traj(:, i) = thetastart + s * (thetaend - thetastart);
         if i>2
            dtraj(:, i) = ds * (thetaend - thetastart);
            ddtraj(:, i) = dds * (thetaend - thetastart);
         end

    end
    traj = traj';
    
end

function [s, ds, dds] = CubicTimeScaling(Tf, t)
    s = 3 * (t / Tf) ^ 2 - 2 * (t / Tf) ^ 3;
    ds = (6 * t) / Tf^2 - (6 * t^2) / Tf^3;
    dds = 6 / Tf^2 - (12 * t) / Tf^3;
end
