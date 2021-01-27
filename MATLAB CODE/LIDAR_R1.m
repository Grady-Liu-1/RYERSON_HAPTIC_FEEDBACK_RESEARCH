function serial_test(d)
%d=[x,y,r,i_out,theta]

%install mapfun(value,fromLow,fromHigh,toLow,toHigh) add-on

%declaring constants
x=1;
y=2;
r=3;
theta=5;

%variables that corespond with haptic motor strength
front = 1;
frontR =1;
frontL=1;
back = -1;

%turns table into matrix, only necessary for test data (since data is in a
%table)
coord = d{:,:};
scatter(coord(:,x),coord(:,y))

%defining arduino variables
frontRPin = 'd6';
frontPin = 'd9' ;
frontLPin = 'd10';
backPin = 'd11';
btnPin = 'a0';
a = arduino('com4','uno'); %%, port, and board
btnState = 1; %% button state


%determine the vibration frequency of the four motors based of data from 4
%different quadrants
for i = 1:length(coord(:,x))
    if (coord(i,x) <= 0.105) && (coord(i,x) >= -0.105)
        if (coord(i,y) > 0)
            if (coord(i,y) < front) 
                front = coord(i,y);
            end
        elseif (coord(i,y) < 0)
        	if (coord(i,y) > back) 
                back = coord(i,y);
            end  
        end
    elseif coord(i,theta)>= 45
        if coord(i,theta)< 90
            if coord(i,r)< frontR
                frontR = coord(i,r);
            end
        elseif coord(i,theta)<= 135
            if coord(i,r)< frontL
                frontL = coord(i,r);
            end
        end
    end
end


vibe = [ 1- frontR, 1 - front, 1 - frontL, 1- (back*-1)];

%{
for i = 1:4
   vibe(i) = mapfun(    vibe(i), 0, 1, 0, 0.66);
   if vibe(i) == 102
       vibe(i) = 0;
   end
end
%}



writePWMDutyCycle(a, frontRPin, 1- frontR);
writePWMDutyCycle(a, frontPin, 1 - front);
writePWMDutyCycle(a, frontLPin, 1 - frontL);
writePWMDutyCycle(a, backPin, 1- (back*-1));

%%below is an infinate loop, stops when btn is pressed
while btnState == 1
    btnState = readDigitalPin(a,'d13');
end






