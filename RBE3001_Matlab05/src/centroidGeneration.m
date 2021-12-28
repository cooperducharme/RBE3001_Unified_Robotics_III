function centroid_point = centroidGeneration(BW)
    BI = regionprops(BW, 'Centroid');
    centroids = BI.Centroid;
    x_coordinant = centroids(:,1);
    y_coordinant = centroids(:,2);
    centroid_point = [x_coordinant,y_coordinant];   
end

