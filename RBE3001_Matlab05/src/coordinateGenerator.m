function [checkerPoint, TCam_To_T0] = coordinateGenerator(TImage_To_Checker, TBase_To_Checker, params, pointToConvert) 
    %Creates the inputs for the worldPoints() function
    T_rot = TImage_To_Checker(1:3,1:3);
    T_trans = TImage_To_Checker(1:3,4);
    
    worldPoints = pointsToWorld(params.Intrinsics, T_rot, T_trans, pointToConvert);
    
    checkerPoint = worldPoints;
    
    worldPointsFullOutput = [worldPoints(1); worldPoints(2); 0; 1];
    
    TCam_To_T0 = TBase_To_Checker * worldPointsFullOutput;
end