%Validierung der kinematisch erfassten Punktwolke

% Die kinematisch erfasste Punktwolke vom Velodyne Laserscanner wurde in dem vorherigen Problem für jeden einzelnen Scan-Block in das Welt/Raum-Koordinatensystem transformiert und somit eine zusammenhängende 3D-Punktwolke gebildet, Abbildung 1. Nachfolgend gilt es die Qualität der kinematischen Punktwolke mit Hilfe einer statisch erfassten 3D-Referenzpunktwolke, Abbildung 2, eines Z+F IMAGER 5016 Laserscanners zu validieren. 
% Für einen vereinfachten Vergleich soll dafür eine einzelne Referenzfläche verwendet werden. Die statisch erfassten Punkte wurden an die Größe der Referenzfläche (TLS_plane.mat) angepasst, siehe Abbildung 3.  Daher müssen in der kinematischen die korrespondierenden Punkte gefunden werden.
% Nachfolgend gilt es folgende Teile zu bearbeiten:
% Punkte in der kinematischen Velodyne Punktwolke finden, welche innerhalb eines vorgegebenen Bereichs (interest_region) liegen, welcher durch die Referenzfläche begrenzt wird
% Schätzen einer ausgleichenden Ebene durch die Referenzpunktwolke und Bestimmung der Ebenenparameter in Form des normierten Normalenvektors 
%     Hinweis: Hierfür gibt es mehrere Möglichkeiten: RANSAC, Singulärwertzerlegung, Gauß-Helmert-Modell etc werden an dieser Stelle jedoch NICHT von Ihnen erwartet! - Verwenden Sie daher einfach eine der in Matlab integrierten Funktionen pcfitplane(). Es sollte dabei beachtet werden, dass für die Ebenenschätzung nicht  zwingend alle Punkte zu berücksichtigen sind, sondern nur die innerhalb  eines gewissen Abstandes zur Ebene. Als Schwellwert kann man sich dabei an der Messunsicherheit des terrestrischen Laserscanners orientieren,  welche mindestens um den Faktor 10 besser ist als beim Velodyne VLP16  Laserscanner.
% Bestimmung der kürzesten Abstände jeden Punktes der kinematischen Punktwolke (welche in der interest_region liegen) zur geschätzten Ebene der Referenzfläche
%     Hinweis: Auch hierfür gibt es mehrere Möglichkeiten der Implementierung. Es reicht aber bereits aus sich der geometrischen Eigenschaft vom Skalarprodukt (Matlab: dot()) und dem Projektionsverfahren zu bedienen 
% Plotten aller berechneten Abstände in einem Histogramm
% Plotten der kinematischen Punktwolke und der statischen Referenzpunktwolke für den Bereich der Referenzfläche (interest_region)


load TLS_plane.mat % Ausschnitt einer statischen Punktwolke für eine Referenzebene: (50000 x 3) double, [X[m], Y[m], Z[m]]
load pcl_velodyne_WKS.mat % Punktwolkendatei als homogene Koordinaten im WKS mit folgendem Aufbau: (4 x 143541) double, [X [m]; Y[m]; Z[m]; 1] 

%Zur Bestimmung der interest_region wird ein Quader mit Hilfe der Referenzdaten aufgespannt: [xmin, xmax, ymin, ymax, zmin, zmax]
interest_region=[min(TLS_plane(:,1)), max(TLS_plane(:,1)), min(TLS_plane(:,2)), max(TLS_plane(:,2)), min(TLS_plane(:,3)), max(TLS_plane(:,3))]; % vorgegebenen Bereich, welcher die Referenzfläche eingrenzt
% ToDo: Identifikation aller Punkte in pcl_velodyne_WKS.mat, welche innerhalb von interest_region liegen.
% Hinweis: Die Variable "pcl_velodyne_WKS_Region" beinhaltet die selektierten homogenen Punktkoordinaten und ist wie folgt definiert: (4 x Anzahl_Punkte_auf_Referenzebene) double, [X[m]; Y[m]; Z[m]; 1] pro Spalte
% Tipp: Entweder find(Bedingung 1 & Bedingung 2 & ...)-Funktion verwenden oder eigene Funktion schreiben zur Filterung. Es ist möglich in einer Zeile alle Punkte zu Filtern oder nacheinander in einer for-Schleife.

x=find(pcl_velodyne_WKS(1,:)>interest_region(1)&pcl_velodyne_WKS(1,:)<interest_region(2));
y=find(pcl_velodyne_WKS(2,:)>interest_region(3)&pcl_velodyne_WKS(2,:)<interest_region(4));
find_z=find(pcl_velodyne_WKS(3,:)>interest_region(5)&pcl_velodyne_WKS(3,:)<interest_region(6));
xy = intersect(x,y);
xyz= intersect(xy,find_z);
pcl_velodyne_WKS_Region = pcl_velodyne_WKS(:,xyz);
Anzahl_Punkte_auf_Referenzebene = size(pcl_velodyne_WKS_Region,2);


% ToDo: Schätzen einer ausgleichenden Ebene durch die Referenzpunktwolke und Bestimmung der Ebenenparameter in Form des normierten Normalenvektors (1 x 3) double Normalenvektor = [n_x, n_y, n_z].
% Hinweis: Verwenden Sie eine der in Matlab integrierten Funktionen (z.B. pcfitplane())
maxDistance_ZF = 0.001;

ptCloud = pointCloud(TLS_plane);
Normalenvektor = pcfitplane(ptCloud,maxDistance_ZF);
Normalenvektor=Normalenvektor.Normal;

Normalenvektor = abs(round(Normalenvektor.*100)/100); %gerundet auf zwei Nachkommastellen für etwaige Rechenungenauigkeiten

% ToDo: Bestimmung der kürzesten Abstände jeden Punktes der kinematischen Punktwolke pcl_velodyne_WKS_Region (welche in der interest_region liegen) zur geschätzten Ebene der Referenzfläche
% Hinweis: Erwartet wird ein (Anzahl_Punkte_auf_Referenzebene x 1) double Vektor mit der Bezeichnung "Abweichungen"
% Tipp: Implementierung möglich mit der Abstandsformel nach Hesse
me_k=abs(round(mean(Normalenvektor*TLS_plane.')*100)/100);
Abweichungen = (Normalenvektor*pcl_velodyne_WKS_Region(1:3,:)-m_k)/sqrt(Normalenvektor(1)^2+Normalenvektor(2)^2+Normalenvektor(3)^2);
Gemittelte_Abweichung = abs(round(mean(Abweichungen)*100)/100); %gerundet auf zwei Nachkommastellen für etwaige Rechenungenauigkeiten

% ToDo: Plotten aller berechneten Abweichungen in einem Histogramm. Achten Sie auf die richtigen Einheiten zur Beschriftung.
figure
hist(Abweichungen,50)
xlabel('Abweichungen [m]')
ylabel('Häufigkeiten')
title('Histogramm der Abweichungen zur Referenzebene')
hold on
% ToDo: Plotten der kinematischen Punktwolke pcl_velodyne_WKS_Region und der statischen Referenzpunktwolke TLS_plane.mat für den Bereich der Referenzfläche (interest_region). Achten Sie auf die richtigen Einheiten zur Beschriftung.
figure
plot3(pcl_velodyne_WKS_Region(1,:).',pcl_velodyne_WKS_Region(2,:).',pcl_velodyne_WKS_Region(3,:).','r')
plot3(TLS_plane(:,1),TLS_plane(:,2),TLS_plane(:,3),'green')
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
title('Kinematische und statische 3D-Punktwolke der Referenzfläche')
legend('Velodyne','Referenz')
hold off