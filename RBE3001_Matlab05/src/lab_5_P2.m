%%
% RBE 3001 Lab 5 example code!
% Developed by Alex Tacescu (https://alextac.com)
%%
clc;
% clear;
% clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;
DEBUG_CAM = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);

%cam = Camera();
cam.DEBUG = DEBUG_CAM;

%% Place Poses per color
purple_place = [150, -50, 11];
green_place = [150, 50, 11];
pink_place = [75, -125, 11];
yellow_place = [75, 125, 11];

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    
    %This will create a transformation matrix from image to checker
    TImage_To_Checker = cam.cam_pose; 
    imshow(snapshot(cam.cam))
    %Creates the inputs for the worldPoints() function
    T_rot = TImage_To_Checker(1:3,1:3);
    T_trans = TImage_To_Checker(1:3,4);
    
    Point_One = [1101 674];
    Point_Two = [577 678];
    Point_Three = [1495 658];
    Point_Four = [1465 428];
    
    worldPoints = pointsToWorld(cam.params.Intrinsics, T_rot, T_trans, Point_Four);
    
    
    % This will create a transformation matrix from the robot base to checker  
    TBase_To_Checker = robot.dh2mat([0, 0, 50, 180]) * robot.dh2mat([-90, 0, -75, 0]); 
    
    %This works as well
    transOfBot = [0 1 0 50; 
                  1 0 0 -75;
                  0 0 -1 0;
                  0 0 0 1];
    
    %Creates the multiplitory array to be used to find the transform of the
    %cam to the base frame.
    worldPointsFullOutput = [worldPoints(1); worldPoints(2); 0; 0];
    
    %Transform of the cam in regaurds to the base frame T0.
    t_Cam_To_T0 = transOfBot .* worldPointsFullOutput;
    
    disp(t_Cam_To_T0);
    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()