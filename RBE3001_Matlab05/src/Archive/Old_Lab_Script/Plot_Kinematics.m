%this file is the implementation of the stick plot for the later part of
%the lab
classdef Plot_Kinematics
    methods
        function plot = plot_arm(obj, arm)
            data_pull = arm.measured_js(1, 0);
            jnt_angles = data_pull(1,:);
            L1 = 95;
            L2 = 100;
            L3 = 100;
            
            %injects joint angles into the zero'd out dh parameters used in
            %fk3001
            dh_parameters = [jnt_angles(1) L1 0 -90; 
                            (jnt_angles(2)-90) 0 L2 0; 
                            (jnt_angles(3)+90) 0 L3 0];
                     
            t_12 = arm.dh2mat(dh_parameters(1,:));
            t_23 = t_12 * arm.dh2mat(dh_parameters(2,:));
            t_34 = t_23 * arm.dh2mat(dh_parameters(3,:));
                         
                      %Start  T_12     T_23      T_34
            link_array = [0 t_12(1,4) t_23(1,4) t_34(1,4);
                          0 t_12(2,4) t_23(2,4) t_34(2,4);
                          0 t_12(3,4) t_23(3,4) t_34(3,4)];
            
            
            %Implements the 3-D plotting of the link array
            plot = plot3(link_array(1,:), link_array(2,:), link_array(3,:), '-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
            title('Pose Representation for Bot');
            xlabel('Distance in X (mm)');
            ylabel('Distance in Y (mm)');
            zlabel('Distance in Z (mm)');
            xlim([-300, 300]);
            ylim([-300, 300]);
            zlim([0, 300]);
            
               
        end 
    end
end