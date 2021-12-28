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
cam = Camera();% Creating the Camera Object

try


    SERV_ID = 1848;            
    SERVER_ID_READ =1910;
    DEBUG   = false;          
    
    disp('Put the objects on the checkerboard, then press any key to continue');
    pause;
    fprintf('\n')
    
    
    %% ------------------------------------------GREEN BALL------------------------------------------
    %% Camera Masking
    %Gets raw image and undistorts
    
    im = snapshot(cam.cam);
    
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
    pause(0.5);
    
    %% ------------------------------------------ORANGE BALL------------------------------------------
    %% Camera Masking
    %Gets raw image and undistorts
    
    im = snapshot(cam.cam);
    
    %Runs the two masks being applied
    disp('Orange Ball');
    M1 = orangeMask(im);       
    erodeI = medfilt2(M1,[30,30]);
    BW = imfill(erodeI, 'holes');

    %% Centroid Generation
    point = centroidGeneration(BW);
    
    %% Demo of Centroid overlaid on Masked Image
    figure (2)
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
    pause(0.5);
    
    %% ------------------------------------------YELLOW BALL------------------------------------------
    %% Camera Masking
    %Gets raw image and undistorts
    
    im = snapshot(cam.cam);
    
    %Runs the two masks being applied
    disp('Yellow Ball');
    M1 = yellowMask(im);       
    erodeI = medfilt2(M1,[30,30]);
    BW = imfill(erodeI, 'holes');

    %% Centroid Generation
    point = centroidGeneration(BW);
    
    %% Demo of Centroid overlaid on Masked Image
    figure (3)
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
    pause(0.5);
    
    %% ------------------------------------------RED BALL------------------------------------------
    %% Camera Masking
    %Gets raw image and undistorts
    
    im = snapshot(cam.cam);
    
    %Runs the two masks being applied
    disp('Red Ball');
    M1 = redMask(im);       
    erodeI = medfilt2(M1,[30,30]);
    BW = imfill(erodeI, 'holes');

    %% Centroid Generation
    point = centroidGeneration(BW);
    
    %% Demo of Centroid overlaid on Masked Image
    figure (4)
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
    pause(0.5);
 
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
    myHIDSimplePacketComs.disconnect();
end

% Clear up mem
%robot.shutdown()
cam.shutdown()