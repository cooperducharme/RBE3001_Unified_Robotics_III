% clear
% clear java
% clear classes;

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

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs); % Creating the Robot Object
% cam = Camera();% Creating the Camera Object

try


    SERV_ID = 1848;            
    SERVER_ID_READ =1910;
    DEBUG   = false;          
    
    disp('Put the objects on the checkerboard, then press any key to continue');
    pause;
    fprintf('\n')
    
    %% ------------------------------------------GREEN BALL------------------------------------------
    %% Camera Masking
    %Gets raw image and resizes for the ball placement
    
    im = snapshot(cam.cam);
%     imshow(im);
%     
%     targetSize = [600 1600];
%     r = centerCropWindow2d(size(im),targetSize);
%     im = imcrop(im, r); 
%     imshow(im)
   
    %Runs the two masks being applied
    disp('Green Ball');
    M1 = greenMask(im);       
    erodeI = medfilt2(M1,[30,30]);
    BW = imfill(erodeI, 'holes');

    %% Centroid Generation
    point = centroidGeneration(BW);
    
    %% Demo of Centroid overlaid on Masked Image
    figure (1)
    imshow(BW);
    hold on
    plot(point(1),point(2),'r*')
    hold off 
    
    %% Coordinate Work:
    %This will create a transformation matrix from image to checker
    TImage_To_Checker = cam.cam_pose; 
    
    TBase_To_Checker = [0 1  0  50; 
                        1 0  0 -100;
                        0 0 -1   0;
                        0 0  0   1];
           
    params = cam.params;
    pointToConvert = point;
    
    [checkerPoints,TCam_To_T0] = coordinateGenerator(TImage_To_Checker, TBase_To_Checker, params, pointToConvert);
    
    %% Outputs the point on the image which is the X and Y Coordinates in terms of the base frame
    disp("X Coordinate: ");
    disp(TCam_To_T0(1));
    disp("Y Coordinate: ");
    disp(TCam_To_T0(2));   
    disp("----------------------");
    
    %% Sending the Robot to the desired position
    
    %Send to ik3001

    
    pp.interpolate_jp([0 0 0], 1000);
    pause(1);
    
    pp.openGripper();
    pause(1);
    
    sendCoordinatesToIk = [TCam_To_T0(1) TCam_To_T0(2)-4 190];
    angles = pp.ik3001(sendCoordinatesToIk);
    disp(angles);
    pp.interpolate_jp([angles(1), angles(2), angles(3)], 1000);  
    
    pause(1);
    pp.interpolate_jp(pp.ik3001([TCam_To_T0(1) TCam_To_T0(2)-4 11]), 1500);  
    pause(2);
    pp.closeGripper();
    pp.interpolate_jp([0 0 0], 1000);
    pause(1);
 
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
    myHIDSimplePacketComs.disconnect();
end

% Clear up mem
pp.shutdown();
% cam.shutdown();