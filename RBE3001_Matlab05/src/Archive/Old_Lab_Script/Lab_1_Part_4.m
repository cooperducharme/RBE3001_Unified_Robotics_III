% RBE3001 - Laboratory 1 
% 
%this file contains code relevant to solving parts 4 and 5 of the lab
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

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs); 


try
%%Main Script for Functions:  
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints
  
  %This will be the array being passed into the csv file
  data = [0 0 0 0;];
  posdata = [0 0 0;];
  %Declaration of Matrix we will use to create .csv:
  writematrix(data, 'Lab_1_Position.csv');
  writematrix(data, 'Lab_1_Position_Part_5.csv');
   %titleDeclaration = ['Time', 'Motor 1', 'Motor 2', 'Motor 3';];
  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single'); 
  
  %% Code for Lab 1 P 5:
  
  %Uses the current position of the arm as point
  %Calibration position values: -90.0000, 85.6600, 33.7100
  
  %First position values: -34.800 48.9400 17.6300
  %Second position values: 3.6000 37.1800 -21.0100
  %Third position values: 59.5200 70.3000 32.9900
  %Fourth position values: -3.6000 60.9400 32.5100
  
  %Actual First position values: -30.4800   43.6600   13.0700
  %Actual Second position values: 1.2000   32.6200  -17.4100
  %Actual Third position values: 51.6000   63.5800   26.7500
  %Actual Fourth position values: -1.9200   54.7000   26.9900
  
  %Fifth position values (W/O interpolation): 59.5200 70.3000 32.9900
  %Actual Fifth position values (W/O interpolation): 60.7200   71.9800   32.9900
  
  %this portion sends the robot to its zero position, pauses, then begins 
  %moving it to the specified position to measure (with interpolation), 
  %as seen above, in positions 1,2,3,4
  pp.interpolate_jp(([0 0 0]), 1000);
  pause(2)
  pp.interpolate_jp(([3.6000 37.1800 -21.0100]), 1000);
  
  %reads the current setpoints and then writes them to the .csv
   tic;
    while toc <= 1 
        ms = toc; 
        posdata = pp.measured_js(1, 0);
        data = [ms posdata(1) posdata(2) posdata(3);];
        writematrix(data, 'Lab_1_Position.csv', 'WriteMode', 'append');   
    end
    %code to arrange csv file
        array=csvread('Lab_1_Position.csv');
        c1 = array(:,1);
        c2 = array(:,2);
        c3 = array(:,3);
        c4 = array(:,4);
  %move arm to start point, now do motion without interpolation to record
  pp.interpolate_jp(([0 0 0]), 1000);
  pause(2)
  pp.interpolate_jp(([3.6000 37.1800 -21.0100]), 0);
  
  %record relevant data
  tic;
    while toc <= 1 
        ms = toc; 
        posdata = pp.measured_js(1, 0);
        data = [ms posdata(1) posdata(2) posdata(3);];
        writematrix(data, 'Lab_1_Position_Part_5.csv', 'WriteMode', 'append');   
    end
        array2=csvread('Lab_1_Position_Part_5.csv');
        d1 = array2(:,1);
        d2 = array2(:,2);
        d3 = array2(:,3);
        d4 = array2(:,4);
                
        %subplots with both sets of data 
        subplot(1,3,1)
        plot(c1,c2, 'r', d1, d2, 'b')
        title('Joint 1')
        xlabel('Time(s)')
        ylabel('Joint Position(deg)')
        %waits for user input to continue
        pause
        subplot(1,3,2)
        plot(c1,c3, 'r', d1, d3, 'b')
        title('Joint 2')
        xlabel('Time(s)')
        ylabel('Joint Position(deg)')
        
        pause
        subplot(1,3,3)
        plot(c1,c4, 'r', d1, d4, 'b')
        title('Joint 3');
        xlabel('Time(s)')
        ylabel('Joint Position(deg)')
        
        %plots figure without non-interpolating data
        figure 
        subplot(1, 3, 1)
        plot(c1,c2,'r')
        title('Joint 1')
        xlabel('Time(s)')
        ylabel('Joint Position(degree)')
        subplot(1,3,2)
        plot(c1,c3,'r')
        title('Joint 2')
        xlabel('Time(s)')
        ylabel('Joint Position(degree)')
        subplot(1,3,3)
        plot(c1,c4,'r')
        title('Joint 3')
        xlabel('Time(s)')
        ylabel('Joint Position(degree)')
%used to make histogram for part 3 of lab
%determines time step between each reading, places in vector hg
%         prev = 0;
%         hg = zeros(length(c1),1);
%         for k = 1:length(c1)
%             hg(k) = (c1(k) - prev);
%             prev = c1(k);
%         end
%         
%         figure
%             histogram(hg);
%             title('Histogram of Time Step');
%             xlabel('Time Step (ms)');
%             ylabel('Occurences');

            
    
    
    
%% Reference Script for Functions: 
%   % Closes then opens the gripper
%   pp.closeGripper()
%   pause(1)
%   pp.openGripper()
%   %pause(1)
%   %pp.servo_jp([30,20,40]);
%   %pause(1)
%   %disp(pp.measured_js(1, 1));
%   pause(1)
%   %disp(pp.goal_js());
%   pp.interpolate_jp(([0,0, 0]), 1000);
%   disp('Made it past the interpolate function');
%   pause(1)
%   %pp.servo_jp([30,20,40]);
%   disp(pp.setpoint_js());

  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

% toc
