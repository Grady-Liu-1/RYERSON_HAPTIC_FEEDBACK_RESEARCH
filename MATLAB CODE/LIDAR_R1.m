function LIDAR_R1()
clear
device = serialport("COM6",9600)
%this code takes lidar data and transmits it to arduino xD
rosshutdown
%IpAddress of Turtlebot
%"http://192.168.1.28";

%Connect to ROS NETWORK Master
MasterIp='192.168.0.233'%ros (server) computer's ip address
rosinit(MasterIp);
tbot=turtlebot('192.168.0.233'); %also server ip address

%


%variables that corespond with haptic motors
front = 1;
frontR =1;
frontL = 1;
back = -1;

%defining arduino variables
frontRPin = 'd6';
frontPin = 'd9' ;
frontLPin = 'd10';
backPin = 'd11';
btnPin = 'a0';

%a = arduino('com4','uno'); %%, port, and board
i = 1
%infinate loop starts here xD
while true
    [scan,scanMsg]=getLaserScan(tbot);
    
    %Must use scanMsg for pull/retrieving data, scan only provides the
    %properties of the matrix
    %Cart holds x,y coordinates of the Lidar Data.
    cart=readCartesian(scanMsg);
    x=cart(:,1);
    y=cart(:,2);
    r=(x.^2+y.^2).^0.5;
    
    %Create the Imaginary Boundary for data pts
    %Let Rmax=1 meter as the calibrated radius
    %Determine value of intensity as a percentage to Rmax
    rmax=0.5;
    rmin=0.1;
    
    %Determine the normalized intensity by replicating the mapping function in
    %arduino: change imax / imin for the arduino side (pwm)
    for k=1:length(r)
        if((r(k,:)>rmin) && (r(k,:)<rmax))
            i_min = 25.5; %low vibration
            i_max = 255; %highest vibration
            i_out(k,:)=((r(k,:)-rmax)*(i_min-i_max)/(rmax-rmin))+i_min;
        elseif((r(k,:)<rmin))
            i_out(k,:)= 255;
        else
            i_out(k,:)=0;
        end
    end
    
    %Determine the angle of the data pt
    ratio=y./x;
    theta=atan2(y,x)*180/pi;
    %normalize data pts that show data pts negative in angle
    for z=1:length(theta)
        if(theta(z,:)<0)
            theta(z,:)=360+theta(z,:);
        end
    end
    
    d=[x,y,r,i_out,theta]
    %x = x coord
    %y = y coord
    %r = normalized distance
    %i_out = converts normalized distance from -1 to 0 - 255
    
    %determine the vibration frequency of the four motors based of data from 4
    %different quadrants
    
    for i = 1:length(d(:,1))
        if (d(i,2) <= 0.105) && (d(i,2) >= -0.105)
            if (d(i,1) < 0)
                if (d(i,4) > front)%front
                    front = d(i,4);
                end
            elseif (d(i,1) > 0)%back
                if (d(i,4) > back)
                    back = d(i,4);
                end
            end
        elseif d(i,5) >= 180 && d(i,5) <= 225
            if d(i,4)> frontL %front left
                frontL = d(i,4);
            end
        elseif d(i,5)<= 180 && d(i,5)>= 135
            if d(i,4)> frontR
                frontR = d(i,4);
            end
        end
    end
    
    
    %front
    %frontL
    %frontR
    %back
    
    
    %vibrates motors connected to arduino
    write(device,"r" + num2str(frontR,'%03.f'),"int8")
    write(device,"f" + num2str(front,'%03.f'),"int8")
    write(device,"l" + num2str(frontL,'%03.f'),"int8")
    write(device,"d" + num2str(back,'%03.f'),"int8")
    
    grid on
    scatter(d(:,1),d(:,2)); drawnow
    hold on
    scatter(0,0)
    line([-1,1],[0.105,0.105])
    line([-1,1],[-0.105,-0.105])
    line([0,-0.7],[0,0.7])
    line([0,-0.7],[0,-0.7]);drawnow
    rectangle('Position',[-1,-1,2,2],'curvature',[1,1],'EdgeColor','green');drawnow
    rectangle('Position',[-0.35,-0.35,2*0.35,2*0.35],'curvature',[1,1],'EdgeColor','red');drawnow
    hold off
    
    clear var d x y r i_out theta i %front frontL frontR back
    
    front = 0;
    frontL = 0;
    frontR = 0;
    back = 0;
    
    
end
rosshutdown
end
