clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

pp = Robot(myHIDSimplePacketComs);
traj = Traj_Planner();

    
try
    %Method Constants
    SERV_ID = 1848;            % we will be talking to server ID 1848 on
    SERVER_ID_READ =1910;% ID of the read packet
    DEBUG   = false;          % enables/disables debug prints 
    
    
    q1 = 0;
    q2 = 0;
    q3 = -90;

    frame_1 = [1 0 0 0;
               0 1 0 0;
               0 0 1 55;
               0 0 0 1];

    frame_2 = [cosd(q1) 0 -sind(q1) 0;
               sind(q1) 0  cosd(q1) 0;
               0         -1  0          40;
               0          0  0          1];

    frame_3 = [cosd(q2-90) -sind(q2-90) 0    100*cosd(q2-90);
               sind(q2-90)  cosd(q2-90) 0    100*sind(q2-90);
               0              0             1          0;
               0              0             0          1];

    frame_4 = [cosd(q3+90) -sind(q3+90) 0    100*cosd(q3+90);
               sind(q3+90)  cosd(q3+90) 0    100*sind(q3+90);
               0              0             1          0;
               0              0             0          1];            

    frame_04 = frame_1*frame_2*frame_3*frame_4;

    lastRow =      [100*cos((pi*q1)/180)*cos((pi*(q2 - 90))/180) + 100*cos((pi*q1)/180)*cos((pi*(q2 - 90))/180)*cos((pi*(q3 + 90))/180) - 100*cos((pi*q1)/180)*sin((pi*(q2 - 90))/180)*sin((pi*(q3 + 90))/180),
                   100*sin((pi*q1)/180)*cos((pi*(q2 - 90))/180) + 100*sin((pi*q1)/180)*cos((pi*(q2 - 90))/180)*cos((pi*(q3 + 90))/180) - 100*sin((pi*q1)/180)*sin((pi*(q2 - 90))/180)*sin((pi*(q3 + 90))/180),
                   95 - 100*cos((pi*(q2 - 90))/180)*sin((pi*(q3 + 90))/180) - 100*cos((pi*(q3 + 90))/180)*sin((pi*(q2 - 90))/180) - 100*sin((pi*(q2 - 90))/180)];

    diffFrameQ1 =  [-100*sin((pi*q1)/180)*cos((pi*(q2 - 90))/180) - 100*sin((pi*q1)/180)*cos((pi*(q2 - 90))/180)*cos((pi*(q3 + 90))/180) + 100*sin((pi*q1)/180)*sin((pi*(q2 - 90))/180)*sin((pi*(q3 + 90))/180), ...
                    100*cos((pi*q1)/180)*cos((pi*(q2 - 90))/180) + 100*cos((pi*q1)/180)*cos((pi*(q2 - 90))/180)*cos((pi*(q3 + 90))/180) - 100*cos((pi*q1)/180)*sin((pi*(q2 - 90))/180)*sin((pi*(q3 + 90))/180), ...
                    0];

    diffFrameQ2 =  [-100*cos((pi*q1)/180)*sin((pi*(q2 - 90))/180) - 100*cos((pi*q1)/180)*sin((pi*(q2 - 90))/180)*cos((pi*(q3 + 90))/180) - 100*cos((pi*q1)/180)*cos((pi*(q2 - 90))/180)*sin((pi*(q3 + 90))/180), ...
                   -100*sin((pi*q1)/180)*sin((pi*(q2 - 90))/180) - 100*sin((pi*q1)/180)*sin((pi*(q2 - 90))/180)*cos((pi*(q3 + 90))/180) - 100*sin((pi*q1)/180)*cos((pi*(q2 - 90))/180)*sin((pi*(q3 + 90))/180), ...
                   -100*sin((pi*(q2 - 90))/180)*sin((pi*(q3 + 90))/180) - 100*cos((pi*(q3 + 90))/180)*cos((pi*(q2 - 90))/180) - 100*cos((pi*(q2 - 90))/180)];

    diffFrameQ3 =  [-100*cos((pi*q1)/180)*cos((pi*(q2 - 90))/180)*sin((pi*(q3 + 90))/180) - 100*cos((pi*q1)/180)*sin((pi*(q2 - 90))/180)*cos((pi*(q3 + 90))/180), ...
                   -100*sin((pi*q1)/180)*cos((pi*(q2 - 90))/180)*sin((pi*(q3 + 90))/180) - 100*sin((pi*q1)/180)*sin((pi*(q2 - 90))/180)*cos((pi*(q3 + 90))/180), ...
                   100*cos((pi*(q2 - 90))/180)*cos((pi*(q3 + 90))/180) + 100*sin((pi*(q3 + 90))/180)*sin((pi*(q2 - 90))/180)];

    zposForFrame1 = [frame_1(1,3), frame_1(2,3), frame_1(3,3)];
    zposForFrame2 = [frame_2(1,3), frame_2(2,3), frame_2(3,3)];
    zposForFrame3 = [frame_3(1,3), frame_3(2,3), frame_3(3,3)];


    returnJacobain = [diffFrameQ1(1)   diffFrameQ2(1)   diffFrameQ3(1);
                      diffFrameQ1(2)   diffFrameQ2(2)   diffFrameQ3(2);
                      diffFrameQ1(3)   diffFrameQ2(3)   diffFrameQ3(3);
                    zposForFrame1(1) zposForFrame2(1) zposForFrame3(1); 
                    zposForFrame1(2) zposForFrame2(2) zposForFrame3(2); 
                    zposForFrame1(3) zposForFrame2(3) zposForFrame3(3)]; 

    %Original Pull test for calculating the jacobian    
    disp(returnJacobain);

    test2 = [diffFrameQ1(1)   diffFrameQ2(1)   diffFrameQ3(1);
             diffFrameQ1(2)   diffFrameQ2(2)   diffFrameQ3(2);
             diffFrameQ1(3)   diffFrameQ2(3)   diffFrameQ3(3)];

    disp(det(test2));
   
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
    myHIDSimplePacketComs.disconnect();
end

pp.shutdown()