%Aufbereitung der Kameradaten

%% Datenvorbereitung: Laden der gegebenen Daten (load)
load('Initial_Camera_Pose.mat')
load('image_points_y.mat')
load('image_points_x.mat')
load('Cb_CornerWorldSystem.mat')

%% Datenvorbereitung für die Bündelblockausgleichung:
% rostime for image_data, n*1 array
image_data.rostime = table2array(Initial_Camera_Pose(:,1));
% camera positon [x,y,z] in global frame for each image, n*3 array, unit mm
image_data.came_pose = table2array(Initial_Camera_Pose(:,2:4));
% camera to world coordinate system rotation (Euler Angle around x, y, z in degrees)                                      
image_data.rvec = table2array(Initial_Camera_Pose(:,5:7));                                               
% camera to world coordinate system translation vector( in the direction of x, y, z in mm)
image_data.tvec = table2array(Initial_Camera_Pose(:,8:10));
% All CheckerboardPoints position [x,y,z] in Global Frame, n*3 unit
image_data.worldPoints = table2array(Cb_CornerWorldSystem);
% x pixel coordinate for all Checkerboard corners of all epoches.
% please note that the first column is rostime and it should excluded.
image_data.imagex = table2array(image_points_x(2:1720,2:337));                  
% y pixel coordinate for all Checkerboard corners of all epoches 
% please note that the first column is rostime and it should excluded.
image_data.imagey = table2array(image_points_y(2:1720,2:337));                    
    
    
%Umspeichern für das bessere MatlabGrader Feedback:

image_data_rostime = image_data.rostime;
image_data_came_pose = image_data.came_pose;
image_data_rvec = image_data.rvec;
image_data_tvec =image_data.tvec;
image_data_worldPoints =image_data.worldPoints;
image_data_imagex = image_data.imagex;
image_data_imagey = image_data.imagey; 