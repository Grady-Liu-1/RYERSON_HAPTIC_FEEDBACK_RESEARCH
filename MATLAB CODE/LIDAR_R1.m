function LIDAR_R1()
    
%IpAddress of Turtlebot
%"http://192.168.1.28";

%Connect to ROS NETWORK Master
MasterIp='192.168.1.26'
rosinit(MasterIp);
tbot=turtlebot('192.168.1.26');

%Create the Imaginary Boundary for data pts 
%Let Rmax=1 meter as the calibrated radius 
%Determine value of intensity as a percentage to Rmax
rmax=1.0;
rmin=0.35;

%variables that corespond with haptic motors
front = 1;
frontR =1;
frontL=1;
back = -1;

%defining arduino variables
frontRPin = 'd6';
frontPin = 'd9' ;
frontLPin = 'd10';
backPin = 'd11';

a = arduino('com4','uno'); %%, port, and board

while(endflag==1%place holder variable x for button shutoff flag)
%Getting Lidar Scan
[scan,scanMsg]=getLaserScan(tbot);

%Must use scanMsg for pull/retrieving data, scan only provides the
%properties of the matrix 
%Cart holds x,y coordinates of the Lidar Data. 
cart=readCartesian(scanMsg);
x=cart(:,1);
y=cart(:,2);
r=(x.^2+y.^2).^0.5;


%Determine the normalized intensity by replicating the mapping function in
%arduino
for k=1:length(r)
    if((r(k,:)>rmin) && (r(k,:)<rmax))
        i_min=0.1;
        i_max=1.0;
        i_out(k,:)=((r(k,:)-rmax)*(i_min-i_max)/(rmax-rmin))+i_min;
    elseif((r(k,:)<rmin))
        i_out(k,:)=1;
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

%determine the vibration frequency of the four motors based of data from 4
%different quadrants
for i = 1:length(d(:,1))
    if (d(i,1) <= 0.105) && (d(i,1) >= -0.105)
        if (d(i,2) > 0)
            if (d(i,2) < front) 
                front = d(i,2);
            end
        elseif (d(i,2) < 0)
        	if (d(i,2) > back) 
                back = d(i,2);
            end  
        end
    elseif d(i,5)>= 45
        if d(i,5)< 90
            if d(i,3)< frontR
                frontR = d(i,3);
            end
        elseif d(i,5)<= 135
            if d(i,3)< frontL
                frontL = d(i,3);
            end
        end
    end
end

%vibrates motors connected to arduino
writePWMDutyCycle(a, frontRPin, 1- frontR);
writePWMDutyCycle(a, frontPin, 1 - front);
writePWMDutyCycle(a, frontLPin, 1 - frontL);
writePWMDutyCycle(a, backPin, 1- (back*-1));

if(btn==1 %button shutoff has been pressed)
    % Ending ROS Server
    rosshutdown
    endflag==1
else
    endflag==0;
end


%End of Loop
end 
%End of Function
end
