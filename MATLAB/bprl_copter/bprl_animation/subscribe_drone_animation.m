server = tcpserver("127.0.0.1",9002);

disp("Visualizer server running")

while true

    if server.NumBytesAvailable > 0

        msg = readline(server);

        data = jsondecode(msg);

        drone_animation_update( ...
            data.x,...
            data.y,...
            data.z,...
            data.roll,...
            data.pitch,...
            data.yaw);

    end

end

function drone_animation_update(x,y,z,roll,pitch,yaw)

persistent fig hg drone combinedobject pathX pathY pathZ

if isempty(fig)

    b=0.6; a=b/3; H=0.06; H_m=H+H/2; r_p=b/4;

    ro = 45*pi/180;
    Ri=[cos(ro) -sin(ro) 0;
        sin(ro) cos(ro) 0;
        0 0 1];

    base_co=[-a/2 a/2 a/2 -a/2;
             -a/2 -a/2 a/2 a/2;
              0 0 0 0];

    base=Ri*base_co;

    to=linspace(0,2*pi);
    xp=r_p*cos(to);
    yp=r_p*sin(to);
    zp=zeros(1,length(to));

    fig=figure('pos',[0 50 800 600]);
    hg=gca;
    view(68,53); grid on; axis equal
    xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([0 3.5]);
    hold on

    drone(1)=patch(base(1,:),base(2,:),base(3,:),'r');
    drone(2)=patch(base(1,:),base(2,:),base(3,:)+H,'r');

    [xc yc zc]=cylinder([H/2 H/2]);

    drone(3)=surface(b*zc-b/2,yc,xc+H/2,'facecolor','b');
    drone(4)=surface(yc,b*zc-b/2,xc+H/2,'facecolor','b');

    drone(5)=surface(xc+b/2,yc,H_m*zc+H/2,'facecolor','r');
    drone(6)=surface(xc-b/2,yc,H_m*zc+H/2,'facecolor','r');
    drone(7)=surface(xc,yc+b/2,H_m*zc+H/2,'facecolor','r');
    drone(8)=surface(xc,yc-b/2,H_m*zc+H/2,'facecolor','r');

    drone(9)=patch(xp+b/2,yp,zp+(H_m+H/2),'c');
    drone(10)=patch(xp-b/2,yp,zp+(H_m+H/2),'c');
    drone(11)=patch(xp,yp+b/2,zp+(H_m+H/2),'c');
    drone(12)=patch(xp,yp-b/2,zp+(H_m+H/2),'c');

    combinedobject = hgtransform('parent',hg);
    set(drone,'parent',combinedobject)

    pathX=[]; pathY=[]; pathZ=[];

end

pathX(end+1)=x;
pathY(end+1)=y;
pathZ(end+1)=z;

plot3(pathX,pathY,pathZ,'b:','LineWidth',1.5)

translation = makehgtform('translate',[x y z]);
rotation1 = makehgtform('xrotate',roll*pi/180);
rotation2 = makehgtform('yrotate',pitch*pi/180);
rotation3 = makehgtform('zrotate',yaw*pi/180);

set(combinedobject,'matrix',translation*rotation3*rotation2*rotation1)

drawnow

end