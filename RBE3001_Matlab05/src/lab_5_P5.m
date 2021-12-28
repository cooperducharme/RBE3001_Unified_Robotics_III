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
traj = Traj_Planner(pp);
cam = Camera();
sort = Sort();

try
    SERV_ID = 1848;            
    SERVER_ID_READ =1910;
    DEBUG   = false;
    
    %Arrays that will become modified in size 
    ball_Coordinates = [];
    start_Points = [];
    end_Points = [];
    colors = [];
    
    %State machine vairables
    find_Ball = 1;
    ball_Sort = 2;
    return_To_Home = 3;
    ball_Sort_Complete = 4;
    placement = 5;
    
    state = find_Ball;
    running = true;
    
    %------------------------------Primer----------------------------------
    TImage_To_Checker = cam.cam_pose; 
    
    TBase_To_Checker = [0 1  0  50; 
                        1 0  0 -100;
                        0 0 -1   0;
                        0 0  0   1];
           
    params = cam.params;
    %----------------------------State machine-----------------------------
    while running
        if find_Ball == state
            disp('Put the objects on the checkerboard, bot starts in one second');
            pause(1);
            fprintf('\n')
            
            %Takes a snapshot of the board
            im = snapshot(cam.cam);
            
            %% Sorting Variables for the ball detection:
            green_State = true;
            yellow_State = true;
            orange_State = true;
            red_State = true;            
            
            %Resets the arrays
            ball_Coordinates = [];
            start_Points = [];
            end_Points = [];
            colors = [];
            
            %% Green Ball Detection:
            %Documentation is same for the orange, yellow, and red masks as
            %well.
            if (green_State)
                %Creates the hsv mask from the green mask function
                mask = greenMask(im);
                %Gets rid of everything except for the subject (in this
                %case the ball).
                erodeI = medfilt2(mask,[30,30]);
                %Fills any part of the ball that was cut out
                BW = imfill(erodeI, 'holes');
                %This is to test if the ball was found. If the non zero 
                %matrix of BW is zero then no ball was detected 
                B = nnz(BW);

                if (B == 0)
                    green_State = false;
                else
                    %If there is a green ball, then a centroid will be
                    %generated:
                    point = centroidGeneration(BW);
                    %For our sorting method, green will be added to the
                    %listo of colors
                    colors = [colors; sort.green];
                    %This will convert our cam point to a point according
                    %to the robot's base
                    [checkerPoints,TCam_To_T0] = coordinateGenerator(TImage_To_Checker, TBase_To_Checker, params, point);
                    %This point is then added to the ball coordinate list
                    ball_Coordinates = [ball_Coordinates; [TCam_To_T0(1), TCam_To_T0(2), 0]];
                end
            end
            
            %% Yellow Ball Detection:
            if (yellow_State)
                mask = yellowMask(im);
                erodeI = medfilt2(mask,[30,30]);
                BW = imfill(erodeI, 'holes');
                B = nnz(BW);

                if (B == 0)
                    yellow_State = false; 
                else
                    point = centroidGeneration(BW);
                    colors = [colors; sort.yellow];
                    [checkerPoints,TCam_To_T0] = coordinateGenerator(TImage_To_Checker, TBase_To_Checker, params, point);
                    ball_Coordinates = [ball_Coordinates; [TCam_To_T0(1), TCam_To_T0(2), 0]];                
                end
            end
            %% Orange Ball Detection:
            if (orange_State)
                mask = orangeMask(im);
                erodeI = medfilt2(mask,[30,30]);
                BW = imfill(erodeI, 'holes');
                B = nnz(BW);

                if (B == 0)
                    orange_State = false;
                else
                    point = centroidGeneration(BW);
                    colors = [colors; sort.orange];
                    [checkerPoints,TCam_To_T0] = coordinateGenerator(TImage_To_Checker, TBase_To_Checker, params, point);
                    ball_Coordinates = [ball_Coordinates; [TCam_To_T0(1), TCam_To_T0(2), 0]];                
                end
            end
            %% Red Ball Detection:
            if (red_State)
                mask = redMask(im);
                erodeI = medfilt2(mask,[30,30]);
                BW = imfill(erodeI, 'holes');
                B = nnz(BW);

                if (B == 0)
                    red_State = false;
                else
                    point = centroidGeneration(BW);
                    colors = [colors; sort.red];
                    [checkerPoints,TCam_To_T0] = coordinateGenerator(TImage_To_Checker, TBase_To_Checker, params, point);
                    ball_Coordinates = [ball_Coordinates; [TCam_To_T0(1), TCam_To_T0(2), 0]];                
                end
            end
            
        %% Sorting State
        state = ball_Sort;
        elseif state == ball_Sort
            
            %Creates start and end points for each ball according to color
            %which is organized in the sort object defined above.
            [start_Points, end_Points] = sort.sort(colors, ball_Coordinates);
            
            %If the end_Point is -1 then it has not been defined within the
            %sort class and therefore has no balls left to deliver.
            if end_Points(1,1) == -1
               disp("Sorting Done!")
               state = ball_Sort_Complete;
            else
                state = placement;
            end
        
        %When the state is placement, traj_planner will be accessed and
        %will execute three functions made for this: movement in the xy
        %plane to the point, movement in the z plane, and a function that
        %will execute this for both the start and end points.
        elseif state == placement
            for i = 1:size(end_Points,1)
                traj.placement(start_Points(i,:), end_Points(i,:));
            end
            disp("Final Placement Coordinates");
            disp(ball_Coordinates);
            state = return_To_Home;
        
        %% Return pose to home for next ball
        elseif state == return_To_Home
            traj.home_Orientation();
            state = find_Ball;
       
        %% Return pose to home and finishing pose
        elseif state == ball_Sort_Complete
            traj.home_Orientation();
            running = false;
        end
    end  

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
    myHIDSimplePacketComs.disconnect();
end

pp.shutdown()