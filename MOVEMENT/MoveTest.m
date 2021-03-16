function MoveTest()
MasterIp='192.168.1.26'
%rosinit('http://192.168.1.120:42425/',"NodeHost",'http://192.168.1.40:58414/');
rosinit('192.168.1.26')

velocity=0.1;
robotCmd=rospublisher("/cmd_vel");
velMsg=rosmessage(robotCmd);



StopROS=0;
while(StopROS~=1)
    keyboard=waitforbuttonpress;
    key_in=double(get(gcf,'CurrentCharacter'));
    if(key_in==119)
       disp("Forward")
       velMsg.Linear.X = velocity;
    elseif (key_in==120)
       disp("Backwards")
       velMsg.Linear.X = -velocity;
    elseif (key_in==97)
       disp("Pivot Left")
       velMsg.Linear.X = 0;
       velMsg.Angular.Z = velocity;
    elseif(key_in==100)
       disp("Pivot Right")
       velMsg.Linear.X = 0;
       velMsg.Angular.Z = -velocity;
    elseif(key_in==115)
       % S -key on keyboard
       disp("Dead Stop")
       velMsg.Linear.X = 0;
       velMsg.Angular.Z = 0;
    end
    
    send(robotCmd,velMsg)
    
    if(key_in==103)
       disp("Ending Program")
       StopROS=1;
    end
    
    
    %Increases velocity on Command -- 30 = uparrow, 31=downarrow
    if(key_in==30)
        velocity=velocity+0.1;
        disp(velocity)
    elseif(key_in==31)
        velocity=velocity-0.1;
        disp(velocity)
    end
    
    
    
end 

% velMsg.Linear.X = velocgity;
% send(robotCmd,velMsg)
% pause(2)
% velMsg.Linear.X = 0;
% velMsg.Angular.Z = -3*velocity;
% send(robotCmd,velMsg)
% pause(10)
% %velMsg.Linear.X = 0;
% velMsg.Angular.Z = 0;
% send(robotCmd,velMsg)
% 
rosshutdown

end
