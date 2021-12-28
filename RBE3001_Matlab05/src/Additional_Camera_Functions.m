classdef Additional_Camera_Functions < handle
    
    properties
       % No globals required yet
    end
    
    methods
       
       %This will act as our primary method for masking the object we are
       %trying to derive the centroid from
       function [BW, maskedRGB] = yellowMask(RGB)
           %Reference for how to create thhe filter:
           %https://www.mathworks.com/matlabcentral/answers/442910-how-to-apply-a-mask-only-on-saturation-in-a-hsv-image 
           hsvIm = rgb2hsv(RGB);
           hIm = hsvIm(:,:,1); % Get the hue
           sIm = hsvIm(:,:,2); % Get the saturation
           vIm = hsvIm(:,:,3); % Get the value
           
           %Have to define channels 1-3 based on the histogram.
           %Use impixelinfo() to have it display the RGB values 
           %as you mouse around the image.
            
           % Define for channel 1 based on histogram 
           channel1Min = 0.136;
           channel1Max = 0.167;

           % Define for channel 2 based on histogram 
           channel2Min = 0.714;
           channel2Max = 1.000;
           
           % Define for channel 3 based on histogram 
           channel3Min = 0.492;
           channel3Max = 0.776;
           
           sliderBW = ((hIm >= channel1Min) && (hIm <= channel1Max) && ...
                       (sIm >= channel2Min) && (sIm <= channel2Max) && ...
                       (vIm >= channel3Min) && (vIm <= channel3Max));
           BW = sliderBW;
           
           maskedRGB = RGB;

           maskedRGB(repmat(~BW,[1 1 3])) = 0;
       end
       
       %This function will generate a centroid around the specified mask
       %value we pass it.
       function centroid_point = centroid_generation(BW)
          BI = regionprops(BW, 'Centroid');
          centroids = BI.Centroid;
          x_coordinant = centroids(:,1);
          y_coordinant = centroids(:,2);
          centroid_point = [x_coordinant,y_coordinant];
       end
       function coordinants = coordinant_generation(point_1, point_2)
           
       end
    end
end