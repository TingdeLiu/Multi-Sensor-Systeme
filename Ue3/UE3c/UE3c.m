%Transformationsmatrizen aufstellen
% Mittels der vorab geschriebenen Funktion rotmat.m werden die vollständigen Transformationsmatrizen in der korrekten Reihenfolge
% aufgestellt. Dazu werden die entsprechenden Transformationsparameter benötigt. Diese sind im Code mit angegeben 
% (pose_PKS, pose_TP). 
% Anschließend müssen die homogenen Transformationsmatrizen aufgestellt und auf die Punktwolke des Velodyne Laserscanners 
% angewendet werden. Da wir es mit mehreren Koordinatensystemen zu tun haben (lokales SKS, PKS, TKS, LKS sowie das WKS), 
% müssen mehrere dieser Transformationen hintereinander durchgeführt werden. Das Ergebnis stellt die transformierte Punktwolke
% des Velodyne Laserscanners im Bezug zum T-Probe Koordinatensystems (TKS) dar. 
% Die einzelnen Transformationen setzen sich aus einer Translation und einer zusammengesetzten Rotation der drei Einzelachsen
% zusammen. Entsprechend müssen verkettete Rotationen angewendet werden. Dabei ist auf die richtige Reihenfolge zu achten ist
% (siehe unten stehende Abbildung).
% Die homogenen 4x4 Transformationsmatrizen (H_PKS, H_TKS) setzen sich nach üblicher Definition aus der Rotationsmatrix, 
% dem Translationsvektor, einer perspektischen Transformation (hier nicht vorhanden) und einem Skalar für die Skalierung 
% (hier 1) zusammen.
% Es werden hier ausschließlich die zeitlich stabilen Transformationen (also vom SKS bis zum TKS) durchgeführt. 
% DIe Berücksichtigung der zeitlich varaiblen Transformationen erfolgt im anschließenden Aufgabenteil je Scan-Block.
% Die Punktwolkendatei pcl_velodyne.mat besteht aus 6 Spalten, welche die GPS-Zeit in Sekunden, die Koordinate in X,Y,Z in Metern, 
% sowie die Reflektivität als auch die Anzahl der vollen Umdrehung angibt, wobei die Spalte 5 und 6 im Rahmen dieser Übung keine
% Anwendung finden. In den Zeilen sind die einzelnen Scanpunkt in zeitlicher Abfolge aufgelistet.
% Die Transformationsparameter vom SKS -------------pose_PKS---------------> PKS --------------pose_TP--------------> TKS 
% sind als 6x1 double angegeben und beinhalten die Translation X;Y;Z und Rotation Omega;Phi;Kappa. Die entsprechenden Einheiten
% für die Translationen sind zu beachten.

load pcl_velodyne.mat % Punktwolkendatei mit folgendem Aufbau: (143541 x 6) double, [Zeit [s], X [m], Y[m], Z[m], Reflektivität[-], Umdrehung [-]] 



% Transformation vom lokalen Sensorkoordinatensystem des Laserscanners zum Plattform-Koordinatensystem: SKS -> PKS (statisch)
pose_PKS=[-0.2211;0.1887;0.0892;-0.025673;0.00022204;-0.00012859]; %6-DoF Transformationsparameter [X[m], Y[m], Z[m], omega[rad], phi[rad], kappa[rad]]

R_PKS =rotmat('x', pose_PKS(4)) * rotmat('z', pose_PKS(6)) * rotmat('y', pose_PKS(5)) ;    % ToDo: Wenden Sie hier die rotmat-Funktion vom vorherigen Problem an und stellen die zusammengesetzte 3D-Rotationsmatrix (3x3-Matrix (double)) um alle drei Achsen auf.
H_PKS =[R_PKS(1,:) pose_PKS(1);R_PKS(2,:) pose_PKS(2);R_PKS(3,:) pose_PKS(3);0 0 0 1] ;   % ToDo: Stellen Sie hier die homogene Transformationsmatrix (4x4-Matrix (double)) unter Einbeziehung von R_PKS und der Translation aus pose_PKS auf
pcl_velodyne_PKS = H_PKS * [pcl_velodyne(:,2:4) ones(size(pcl_velodyne(:,2:4),1),1)].' ;    % ToDo: Wenden Sie die homogene Transformationsmatrix H_PKS auf die gesamte lokale 3D-Punktwolke (aus pcl_velodyne.mat) vom Velodyne Laserscanner an 

% Transformation vom Plattform-Koordinatensystem zum T-Probe-Koordinatensystem: PKS -> TKS (statisch)
pose_TP=[93.2665;22.9625;-468.9089;-0.0036666;1.5736;-0.023157]; %6-DoF Transformationsparameter [X[mm], Y[mm], Z[mm], omega[rad], phi[rad], kappa[rad]]
R_TKS =  rotmat('x', pose_TP(4)) * rotmat('z', pose_TP(6)) * rotmat('y', pose_TP(5));  % ToDo: Wenden Sie hier die rotmat-Funktion vom vorherigen Problem an und stellen die zusammengesetzte 3D-Rotationsmatrix (3x3-Matrix (double)) um alle drei Achsen auf.
H_TKS =  [R_TKS(1,:) pose_TP(1);R_TKS(2,:) pose_TP(2);R_TKS(3,:) pose_TP(3);0 0 0 1] ;  % ToDo: Stellen Sie hier die homogene Transformationsmatrix (4x4-Matrix (double)) unter Einbeziehung von R_TP und der Translation aus pose_TP auf - Tipp: auf Einheitenunterschiede im Code achten!
pcl_velodyne_TKS = H_TKS * pcl_velodyne_PKS ;  % ToDo: Wenden Sie die homogene Transformationsmatrix H_TKS auf die sich im PKS befindliche gesamte 3D-Punktwolke vom Velodyne Laserscanner an

function R = rotmat(axis, angle)
%Input:
% - axis <-- beinhaltet einen Char mit der Angabe der Achse. Also: 'x', 'y' oder 'z'
% - angle <-- beinhaltet den Rotationswinkel in der Einheit Radiant

%To-Do: Berechnen der Rotationsmatrix 'R' mit Fallunterscheidung für die Achsauswahl. 
%Die Rotationsmatrixen 'R' sollen so wie in Abb. 1 des Aufgabentextest dargestellt berechnet werden.

if axis == 'x'
   R = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];
elseif axis == 'y'
   R = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
elseif axis == 'z'
  R = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
end

end
