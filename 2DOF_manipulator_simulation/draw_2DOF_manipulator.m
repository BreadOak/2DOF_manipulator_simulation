

function draw_2DOF_manipulator(th1_list,th2_list,l1,l2)
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