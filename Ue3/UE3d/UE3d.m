%Individuelle Georeferenzierung jeden Scan-Blocks

% Die vorherigen Transformationen der Punktwolke waren rein statisch und für alle gemessenen Punkte des Laserscanners
% (pcl_velodyne.mat) identisch. Nun gilt es für jeden einzelnen Scan-Block (siehe erstes Problem) eine individuelle 
% Transformation durchzuführen, da sich das Roboter-Fahrzeug im Labor bewegt (siehe unten angefügter Plot). 
% Die entsprechenden Posen-Parameter dafür werden durch das Verfolgen mit dem Lasertracker zur Verfügung gestellt. 
% Jedoch werden die Messwerte vom Lasertracker (LT_pose.mat) "nur" mit 50Hz zur Verfügung gestellt und liegen somit 
% nicht zu den individuellen Zeitpunkten der einzelnen Scan-Blöcke vor. Aus diesem Grund müssen für jeden Scan-Block 
% die zugehörigen kinematischen Transformationsparameter interpoliert und auf den mittleren Zeitpunkt des Scan-Blocks 
% bezogen werden.
% Insgesamt müssen folgende Teile bearbeitet werden:
%     Schleife über alle Scan-Blöcke aufstellen
%     Mittleren Zeitpunkt eines Scan-Blocks bestimmen
%     Identifikation des nächst kleineren und nächst größeren Zeitpunkt der verfügbaren Posen-Informationen des Lasertrackers
%     Lineare Interpolation der LT-Pose für den mittleren Zeitpunkt des Scan-Blocks
%     Aufstellen der homogenen Transformationsmatrix auf Basis der interpolierten Pose (unter Beachtung der unten angegebenen  
% Reihenfolge der Rotationen)
%     Transformation der 3D-Koordinaten des Scan-Blocks vom T-Probe-Koordinatensystem (TKS) ins Lasertracker-Koordinatensystem 
% (LKS) (unter Beachtung der unten angegebenen Reihenfolge der Rotationen)
%     Transformation ins Welt/Raum-Koordinatensystem aufstellen und durchführen (unter Beachtung der unten angegebenen
% Reihenfolge der Rotationen)
% Das Ergebnis von diesem Problem soll die georeferenzierte Punktwolke im WKS sein, siehe Abbildung 3. In Abbildung 2 
% dargestellt ist,  die Punktwolke des Labors des Velodyne VLP16 im SKS. In dieser können zwar Strukturen des 3D-Labors 
% erkannt werden, aber diese sind noch falsch im Raum angeordnet.
% Hinweis: Für diesen Aufgabenteil werden Zwischenlösungen aus den vorherigen Aufgabenteilen benötigt. Neben der Funktion 
% rotmat.m umfasst dies auch die Datensätze zur Velodynepunktwolke (original mit Zeitpunkten, sowie im TKS), sowie die 
% detektierten Scan-Blöcke. Um Unabhängigkeit zu erzeugen, wurden im Vergleich zu den anderen Teilaufgaben hier veränderte
% Datensätze verwendet. DIes ändert nichts an der Vorgehensweise, sollte einem nur bewusst im Falle möglicher Abweicungen 
% zu vorherigen Lösungen sein.

%----------------------------------------------------------------------------------------

load pcl_velodyne.mat % Originäre Punktwolkendatei im SKS mit folgendem Aufbau: (136124 x 6) double, [Zeit [s], X [m], Y[m], Z[m], Reflektivität[-], Umdrehung [-]] 
load pcl_velodyne_TKS.mat % Punktwolkendatei als homogene Koordinaten im TKS mit folgendem Aufbau: (4 x 136124) double, [X [m]; Y[m]; Z[m]; 1] 
load LT_pose.mat % 6-DoF Pose vom Lasertracker gemessen zur T-Probe mit folgendem Aufbau: (2263 x 7) double, [Zeit [s], X [mm], Y [mm], Z [mm], omega [rad], phi [rad], kappa [rad]]
load pcl_time_bloecke.mat % IDs im Bezug auf pcl_velodyne.mat für die einzelnen Scan-Blöcke mit folgendem Aufbau: (1 x 9901) cell, pcl_time_bloecke{1,1:9901}

% ToDo: Schleife über alle Scan-Blöcke aufstellen mit folgenden Teilaufgaben
num = 0;
for b = 1 : length(pcl_time_bloecke)
    % ToDo: Mittlerer Zeitpunkt für den Scan-Block b unter Verwendung von pcl_velodyne.mat und pcl_time_bloecke.mat
    t_b = length(pcl_time_bloecke{1,b});
    num = t_b+num; 
    time(b) = sum(pcl_velodyne(num-t_b+1:num,1))/t_b;
    
    % ToDo: Finden des nächst kleineren und nächst größeren Zeitpunkts im Datensatz der Lasertracker-Pose (bezogen auf den mittleren Zeitpunkt für den aktuellen Scan-Block)
    % Hinweis: Überprüfen Sie, ob überhaupt sowohl ein kleinerer als auch ein größerer Zeitpunkt in LT_pose verfügbar sind. Falls nicht, beenden Sie sofort die in Zeile 7 startende for-Schleife.
    LT_time_kl(b) =length(LT_pose(LT_pose(:,1)<time(b)));% Zeile in LT_pose(:,1) mit dem nächst kleineren Zeitpunkt zu time(b)
    LT_time_gr(b) =length(LT_pose(LT_pose(:,1)>time(b)));% Zeile in LT_pose(:,1) mit dem nächst größeren Zeitpunkt zu time(b)
    
    if LT_time_kl(b)==0||LT_time_gr(b)==0
        break;
    end
    
    % ToDo: Lineare Interpolation der Posenparameter vom Lasertracker auf den mittleren Zeitpunkt für den aktuellen Scan-Block
    % Hinweis: Verwenden Sie z.B. den Matlab-Befehl interp1()
    LT_pose_interp(1:7,b) = interp1(LT_pose(:,1),LT_pose(:,1:7),time(b)).';% LT_pose_interp soll als Output pro Spalte die folgenden Parameter haben: [GPS-Zeit [s]; X [mm]; Y [mm]; Z [mm]; omega [rad]; phi [rad]; kappa [rad]]
    
    % ToDo: Aufstellen der homogenen Transformationsmatrix auf Basis der aktuell gültigen Transformationsparameter aus LT_pose_interp(1:7,b) (verwenden Sie die bereits geschriebene Funktion rotmat.m)
    % Hinweis: Beachten Sie die oben angegebene veränderte Reihenfolge der Rotationen im Vergleich zu den Rotationen vom SKS zum TKS
    R_LKS =rotmat('z', LT_pose_interp(7,b))*rotmat('y', LT_pose_interp(6,b))*rotmat('x', LT_pose_interp(5,b));
    H_LKS =[R_LKS LT_pose_interp(2:4,b)*10^(-3);0,0,0,1];
    pcl_velodyne_LKS = H_LKS * pcl_velodyne_TKS(:, pcl_time_bloecke{b});
    
    % Transformation vom Lasertracker-Koordinatensystem zum Welt/Raum-Koordinatensystem: LKS -> WKS (statisch)
    pose_WKS=[15191.0322;12413.8552;1405.5945;-0.0019752;0.0011798;3.0729]; %6-DoF Transformationsparameter [X[mm], Y[mm], Z[mm], omega[rad], phi[rad], kappa[rad]]
    R_WKS = rotmat('z', pose_WKS(6))*rotmat('y', pose_WKS(5))*rotmat('x', pose_WKS(4));   % ToDo: Wenden Sie hier die rotmat-Funktion vom vorherigen Problem an und stellen die zusammengesetzte 3D-Rotationsmatrix (3x3-Matrix (double)) um alle drei Achsen auf. Hinweis: Beachten Sie die oben angegebene veränderte Reihenfolge der Rotationen im Vergleich zu den Rotationen vom SKS zum TKS
    H_WKS =[R_WKS pose_WKS(1:3)*10^(-3);0,0,0,1];    % ToDo: Stellen Sie hier die homogene Transformationsmatrix (4x4-Matrix (double)) unter Einbeziehung von R_WKS und der Translation aus pose_WKS auf
    pcl_velodyne_WKS(:, pcl_time_bloecke{b}) = H_WKS * pcl_velodyne_LKS;
end

time_ScanBlock_1 = time(1); % Zur Überprüfung ob der erste gemittelte Zeitpunkt korrekt ist.
LT_pose_interp_1 = LT_pose_interp(1:7,1); % Zur Überprüfung ob die ersten gemittelten Transformationsparameter korrekt sind.

function R = rotmat(axis, angle)
% ToDo: Fertige Funktion aus vorherigem Problem hier einfügen
 switch axis
    case 'x'
        R=[1,0,0;0,cos(angle),-sin(angle);0,sin(angle),cos(angle)];
    case 'y'
        R=[cos(angle),0,sin(angle);0,1,0;-sin(angle),0,cos(angle)];
    case 'z'
        R=[cos(angle),-sin(angle),0;sin(angle),cos(angle),0;0,0,1];
    otherwise
        disp('pls use xyz')
end   
end
