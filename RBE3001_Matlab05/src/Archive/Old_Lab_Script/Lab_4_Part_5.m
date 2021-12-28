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
plot_arm = Plot_Kinematics();
    
try
    %Method Constants
    SERV_ID = 1848;            % we will be talking to server ID 1848 on
    SERVER_ID_READ =1910;% ID of the read packet
    DEBUG   = false;          % enables/disables debug prints 
        
    %Time variables:
    syms t;
    t;
    
    %Creates 30 different times ranging by 0.1 from each consecuative time
    time_step_inc = 0:0.1:3;

    %Data arrays for position, velocity, and acceleration
    angles = [];
    angle_data = [];
    angular_vel = [];
    scalar_vel = [];
    linearVelocity = [];
   
    %Creating goal points of triangle:
    goal_1 = [40 60 20];
    goal_2 = [20 -50 30];
    goal_3 = [12 60 60];
    
    goal = [goal_1;goal_2;goal_3;goal_1];
    
    
    %Initial movement to set to zero with interpolation of three seconds
    pp.interpolate_jp([0 0 0], 1);
    pause(1);
    
    %Creates angles from position
    init_angles = pp.ik3001(goal_1);
    %Goes to the first angle
    pp.interpolate_jp(init_angles, 1);
    pause(1);
    
    %For each of the goals without considering the above initial angle
    for i=1:size(goal)-1
        
        %Creates the x, y, and z coefficients from quintic trajectory
        %for the formation of their quintic polynomials
        x_coeff = traj.cubic_traj(0,3,0,0,goal(i,1),goal(i+1,1));
        y_coeff = traj.cubic_traj(0,3,0,0,goal(i,2),goal(i+1,2));
        z_coeff = traj.cubic_traj(0,3,0,0,goal(i,3),goal(i+1,3));

        
        %Linear position equations
        x_pos = x_coeff(1) + x_coeff(2)*t + x_coeff(3)*(t^2) + ...
            x_coeff(4)*(t^3);
        
        y_pos = y_coeff(1) + y_coeff(2)*t + y_coeff(3)*(t^2) + ...
            y_coeff(4)*(t^3);
        
        z_pos = z_coeff(1) + z_coeff(2)*t + z_coeff(3)*(t^2) + ...
            z_coeff(4)*(t^3);

        
        %Supstitution for the time step (in this case we have 30 steps for
        %each position we move to. We will pass this to ik3001 for our
        %angles.
        X_traj_pos = subs(x_pos,t,time_step_inc);
        Y_traj_pos = subs(y_pos,t,time_step_inc);
        Z_traj_pos = subs(z_pos,t,time_step_inc);

        
        for j=1:30
            %% Live Plot Graph:
            figure(1);
            plot_arm.plot_arm(pp);
            hold on
           
            %% Move Bot P1:
            %All of these positions are derivative from the 30 positions
            %created through the time step.
            pass_to_ik = [X_traj_pos(j),Y_traj_pos(j),Z_traj_pos(j)];
            
            %% Quiver Graph:
            % ALL of this is for graphing position:
            
            %Get the live values measured from robot
            pose = pp.measured_js(1,1);
            
            %Split the pose into 3 separate parts
            x_pose = pose(1,1);
            y_pose = pose(1,2);
            z_pose = pose(1,3);
            
            x_vel = pose(2,1);
            y_vel = pose(2,2);
            z_vel = pose(2,3);
            
            pose = [x_pose y_pose z_pose];
            velocity = [x_vel y_vel z_vel];
            
            velocity_jacobian = pp.fdk3001(pose, velocity);
            quiver_unit_v = [velocity_jacobian(1), velocity_jacobian(2), velocity_jacobian(3)];
            q = quiver3(pass_to_ik(1),pass_to_ik(2),pass_to_ik(3),quiver_unit_v(1),quiver_unit_v(2),quiver_unit_v(3));
            hold off
            
            %Records the linear velocities
            linearVelocity = [linearVelocity; [velocity_jacobian(1) velocity_jacobian(2) velocity_jacobian(3)]];
            
            %Records the angular velocities
            angular_vel = [angular_vel; [velocity_jacobian(4) velocity_jacobian(5) velocity_jacobian(6)]];
            
            %Records the scalar velocities
            scalar_conversion = norm(linearVelocity);
            scalar_vel = [scalar_vel; scalar_conversion];
            
            
            %% Move Bot P2:
            %After graphing is prioritized, we then adjust the bot to the
            %proper angles for the 30 that were created through
            %quintic_traj above.
            angles = pp.ik3001(pass_to_ik);
            angle_data = [angle_data; [angles(1) angles(2) angles(3)]];
            pp.interpolate_jp(angles,0);
                        
        end  
    end
        
        %x = linspace(0, 9, 90);
        %% Plotting:
        % Plotting the linear, angular, and scalar velocities
        disp(linearVelocity);
        disp(angular_vel);
        
        x = linspace(0,9,90);

        figure(2)
        subplot (3,1,1)   
        plot(linearVelocity(:,1));
        hold on
        plot(linearVelocity(:,2));
        hold on 
        plot(linearVelocity(:,3));
        title("Linear Velocity vs Time")
        ylabel("Linear Velocity (mm/s)");
        xlabel("Time(s)");
        legend("X Axis Data", "Y Axis Data", "Z Axis Data");
        hold off
        
        
        subplot(3,1,2)
        plot(x, angular_vel(:,1));
        hold on 
        plot(x, angular_vel(:,2));
        hold on
        plot(x, angular_vel(:,3));
        title("Angular Velocity vs Time")
        ylabel("Angular Velocity (radians/second)");
        xlabel("Time(s)");
        legend("X Axis Data", "Y Axis Data", "Z Axis Data");
        hold off
                
        subplot(3,1,3)
        plot(x, scalar_vel);
        title("Scalar Velocity vs Time")
        ylabel("Scalar Velocity (mm/s)");
        xlabel("Time(s)");
        legend("Scalar Velocity Data");
        hold off
                
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
    myHIDSimplePacketComs.disconnect();
end

pp.shutdown()