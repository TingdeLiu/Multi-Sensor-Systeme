%Verbesserung der Kamera-Pose, mit Hilfe einer Bündelblockausgleichung
% Schritt 1:
%     Zuerst sollen in diesem Abschnitt  drei Datensätze für die Bündelblockausgleichung in das Workspace importiert:
% intrinsischen Kamerakalibrierungsparameter, welche in der folgenden mat-Datei gespeichert sind 
%                 cameraparams.mat
%         2. Die Datenstruktur, welche in der vorherigen Problemstellung erstellt wurde (unabhängig der individuellen Lösung)
%                 image_data.mat
%         3. Die erfassten Ecken des Schachbrettmusters für alle gespeicherten Einzelbilder
%                         PT1.mat
%                         PT2.mat
%             die zwei PTX.mat-Dateien beinhalten Objekte des Matlab-Klassentyp pointTrack. 
%             Diese sollen in der Struktur image_data zusammengeführt werden.
% 
% Schritt 2:
% Vervollständigen des Code, mithilfe der beigefügten Kommentare.


%% import the data here
load('PT2.mat')
load('PT1.mat')
load('image_data.mat')
load('cameraParams.mat')


%% Merging the PT1 and PT2 in to pointTracks variable 
    pointTracks(1:160) = PT1;
    pointTracks(161:336) = PT2;
  
%% Save the pointTracks variable in a new field with the same name in the image_data struct:
   image_data.pointTracks = pointTracks;
    
%% The following function with two inputs uses other functions and calculate the refined camera Poses  
 image_data_refined = bundel_adjustment(image_data,cameraParams); % Rufe die unten implementierte Funktion mit den verfügbaren Daten auf

%% Functions

    function image_data_new = bundel_adjustment(image_data,cameraParams)
        cameraPoses = getcameraPoses(image_data); % camera initial Poses
        [xyzRefinedPoints,refinedPoses] = bundleAdjustment(image_data.worldPoints,image_data.pointTracks,cameraPoses,cameraParams.Intrinsics); % Function's inputs should be inserted 
        
         for i=1:length(refinedPoses.AbsolutePose)
             Position_new(i,:)= refinedPoses(i,:).AbsolutePose.Translation; % This line extract the new camera Poses
         end
         
        image_data_new=image_data;
        image_data_new.came_pose=Position_new;
    end


    %% This function creat a initial camera Pose object compatible with BA function's input
    
    function cameraPoses = getcameraPoses(data)
        for i=1:size(data.rvec,1)
            ViewId(i,1)=uint32(i); %值 UInt32 类型表示值范围为 0 到 4，294，967，295 的无符号整数
            AbsolutePose(i)=rigid3d(rotationVectorToMatrix(data.rvec(i,:))',data.came_pose(i,:)); 
        end
        AbsolutePose=AbsolutePose';
        cameraPoses=table(ViewId,AbsolutePose);
    end

