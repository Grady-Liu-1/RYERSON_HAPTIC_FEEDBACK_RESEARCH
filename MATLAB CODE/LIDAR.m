function LIDAR()
    
%IpAddress of Turtlebot
%"http://192.168.1.28";

%Connect to ROS NETWORK Master
MasterIp='192.168.1.26'
rosinit(MasterIp);
tbot=turtlebot('192.168.1.26');
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
rmax=1.0;
rmin=0.35;

%Determine the normalized intensity by replicating the mapping function in
%arduino
for k=1:length(r)
    if((r(k,:)>rmin) && (r(k,:)<rmax))
        i_min=0.1;
        i_max=1.0;
        i_out(k,:)=((r(k,:)-rmax)*(i_min-i_max)/(rmax-rmin))+i_min;
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

%Determine the quadrant where to send vibration intensity to arduino
%Determine how many quadrants there will be 
%Divide data points to n-quadrants, then take max i_out(intensity) value for the
%quadrant necessary and send that value to the arduino

d=[x,y,r,i_out,theta]



%Denote Values for distance as robot approaches imaginary Boundary 
%Create method to populate points with a color scale indicating intensity

%Determine method to remove noise from the dataset.

%if n-quadrants=4, create while-loop to look for minimum r value and
%corresponding i_out to arduino for feedback to glove.


plot(scanMsg)
rosshutdown
end
