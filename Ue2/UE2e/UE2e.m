%Systemkalibrierung der Kamera und der IMU

%  In diesem Abschnitt geht es um die Schätzung der Systemkalibrierungsparameter (Transformation) zwischen IMU und Kamera.   
% 
% Die obige Abbildung zeigt die Trajektorie der IMU in blau, die Trajektorie der Kamera in rot und die Schachbrettecken in grün im globalen Koordinatensystem.
% 
% Die folgende Abbildung zeigt die schematische Darstellung der Koordinatensysteme im Verhältnis zueinander. Dabei steht I für IMU, G für Global und K für die Kamera.
% 
% Die MATLAB-Datei Transformation_data.mat enthält die Transformationsparameter von Global zu IMU (G2I) und Kamera zu Global (K2G).
% 
% Aufgabe 1: Schätzen Sie die Rotationsparameter zwischen Kamera und IMU für jede Epoche. Die Werte sollten einem nx3 Matlab-Array zugewiesen werden, wobei n die Anzahl der Epochen und 3 die Rotationen um die Z-, Y- und. X-Achse ist. Die Einheiten sollten in Grad angegeben werden.            Variablename: C2IRotation
% Aufgabe 2: Schätzen der Kamera-zu-IMU-Translationsparameter für jede Epoche. Die Werte sollten einem nx3-Matlab-Array zugewiesen werden, wobei n die Anzahl der Epochen und 3 Translationen etlang X-, Y- und X-Achse ist. 
% Variablename: C2ITranslation
% Aufgabe 3: Plotten Sie die Rotationswinkel in Grad um 3 Achsen gegen die Zeit in einer Abbildung.   
% Aufgabe 4: Plotten Sie die Translation entlang von 3 Achsen gegen die Zeit.
% Aufgabe 5: Berechnen Sie den Mittelwert der Rotationswinkel (1x3 Array) und der Translationen (1x3 Array).            Variablename: RotationAngle_mean   & Translation_mean
% Aufgabe 6: Berechnung der Standardabweichung von Rotationswinkeln (1x3 Array) und Translationen (1x3 Array).             Variablename: RotationAngle_std  & Translation_std
% 
% ***HINWEIS: 
% Die Rotationswinkel sind in Radiant angegeben und ihre Reihenfolge ist 'ZYX'. Bitte achten Sie darauf, wenn Sie die Rotationsmatrix berechnen.
% Die Translationen sind in Meter angegeben.
% Achten Sie auf die Namen der Variablen. Die Namen der Variablen kann man auch in der Assessment finden.
% Vergessen Sie nicht, Ihre Achsen richtig zu beschriften und eine Legende für Ihre Diagramme zu erstellen.
% Tipp: Um die Kalibrierungsparameter zu berechnen, brauchen Sie nur an eine 3D-Transformation zwischen Koordinatensystemen zu denken.



% Importing the data
load('Transformation_data.mat')
%Initialisiseren der benötigten Matrizen:
sizeTransfData = size(Transformation_data.G2IRotation);
C2IRotation = zeros(sizeTransfData);
C2ITranslation = zeros(sizeTransfData);

% Calculating the calibration paramter for each epoch:
for i = 1:size(Transformation_data.Rostime)
    R_kg = Transformation_data.K2GRotation(i,:);
    R_gi = Transformation_data.G2IRotation(i,:);

    c_k_g =  eul2rotm(R_kg); % die Rotationsmatrix                
    c_g_i = eul2rotm(R_gi); % die Rotationsmatrix
    
    C2IRotation(i,:) = rad2deg(rotm2eul(c_g_i*c_k_g));
    C2ITranslation(i,:) = Transformation_data.G2ITranslation(i,:) + (c_g_i*(Transformation_data.K2GTranslation(i,:).')).' ;
end



% Plotting the calibration parameters against the time:

time=Transformation_data.Rostime;
C2Ia_x=C2IRotation(:,1);
C2Ia_y=C2IRotation(:,2);
C2Ia_z=C2IRotation(:,3);
figure('Name','Rotationswinkel','NumberTitle','off')
hold on
plot(time,C2Ia_x)
plot(time,C2Ia_y)
plot(time,C2Ia_z)
xlabel('rostime (Sec)')
ylabel('Rotationswinkel (degree)')
legend('C2IRotation_x','C2IRotation_y','C2IRotation_z')

C2It_x=C2ITranslation(:,1);
C2It_y=C2ITranslation(:,2);
C2It_z=C2ITranslation(:,3);

figure('Name','Translationen','NumberTitle','off')
hold on
plot(time,C2It_x)
plot(time,C2It_y)
plot(time,C2It_z)
xlabel('rostime (Sec)')
ylabel('Translationen (Meter)')
legend('C2ITranslationen_x','C2ITranslationen_y','C2ITranslationen_z')

% Calculating the mean (mean) value and standard deviation (std) of each transforamtion component:

RotationAngle_mean = mean(C2IRotation);
RotationAngle_std = std(C2IRotation);
Translation_mean = mean(C2ITranslation);
Translation_std = std(C2ITranslation);