% Einzelne Scan-Blöcke identifizieren
% Der verwendete Velodyne VLP16 Laserscanner besitzt 16 individuelle Lasereinheiten. Diese werden nicht exakt zeitgleich 
% betrieben, sondern mit einem zeitlichen Versatz von 2,304 µs. Wenn der 16te Laserstrahl ausgesendet wurde erfolgt eine 
% Pause von 18,432 µs bevor der nächste so genannte Scan-Block gestartet wird. Ein komplette Scanblock benötigt somit 55,296 µs 
% insgesamt. Auf der folgenden Abbildung ist dieser Zusammenhang nochmal skizziert.
% 
% [Auszug aus dem Manual von Velodyne, Seite 50]
% 
% Jeder einzeln gemessene 3D-Punkt bekommt einen eigenen GPS-Zeit-Stempel zugeordnet. Um die Effizienz des Algorithmus
% zu steigern wird nicht jeder gemessene 3D-Punkt individuell transformiert. Daher werden die Transformationen im weiteren
% Verlauf immer pro Scan-Block durchgeführt. Dafür wird der gemittelte Zeitpunkt dieses Blockes verwendet. 
% Die Ermittlung der Scan-Blöcke erfolgt anhand der charakteristischen Pause von 18,432µs. Diese wird auf Basis der 
% Zeitstempel (Spalte 1 von pcl_velodyne.mat) durchgeführt. Dafür wird die die gerundete Variable DELTA_T = 18e-6 vorgegeben.
% Abgespeichert werden die gebildeten Scan-Blöcke in dem Cell-Array pcl_time_bloecke. Zur Überprüfung wird abschließend die 
% Länge des Cell-Arrays, also die Anzahl aller Scan-Blöcke mit der Referenzlösung validiert.
% 
% Die Punktwolkendatei pcl_velodyne.mat besteht aus 6 Spalten, welche die 
% ["GPS-Zeit in Sekunden",  die Koordinate in "X", "Y", "Z" in Metern, sowie die "Reflektivität" und die "Anzahl der vollen
% Umdrehung"] angibt.
% Die Spalte 5 und 6 werden im Rahmen dieser Übung keine Anwendung finden. In den Zeilen sind die einzelnen Scanpunkt 
% in zeitlicher Abfolge aufgelistet. (Zur Information: es handelt sich hierbei um eine ausgedünnte Punktwolke, welche nicht
% mehr alle ursprünglichen Scan-Blöcke enthält. Die eigentliche Punktdichte ist somit wesentlich höher, spielt für 
% diese Übung jedoch keine Rolle)
% Gesucht ist das cell-array pcl_time_bloecke mit der Dimension 1xAnzahl_Bloecke, welches jeweils nur die IDs mit Bezug 
% auf die Zeile in pcl_velodyne.mat beinhaltet. Als Ergebnis werden hier noch keine Zeitpunkte gesucht!

% ----------------------------------------------------------------------------------------------------------------------

load pcl_velodyne.mat % Punktwolkendatei mit folgendem Aufbau: (143541 x 6) double, [Zeit [s], X [m], Y[m], Z[m],
% Reflektivität[-], Umdrehung [-]] 
DELTA_T = 18e-6; %Signifikanter Zeitsprung zwischen zwei Scan-Blöcken laut Datenblatt vom Hersteller 
% (gerundeter Wert für den Fall von Zeitungenauigkeiten)

%ToDo: Identifikation der einzelnen Scan-Blöcke mit Messungen (3D-Koordinaten) der maximal 16 Laserstrahlen
pcl_time_bloecke = cell(0); %Initialisierung des gesuchten Cell-Arrays mit der Dimension (1 x Anzahl_Bloecke), 
% welches jeweils die IDs (bzw Zeilen) der Elemente pro Scan-Block beinhaltet



for i = 2:size(pcl_velodyne,1)
    timedifference = pcl_velodyne(i,1) - pcl_velodyne(i-1,1);

    if timedifference > DELTA_T
         pcl_time_bloecke = [pcl_time_bloecke i];
    end

end



Anzahl_Bloecke = size(pcl_time_bloecke,2) % Für die Überprüfung wird die Dimension hinsichtlich 
% der Anzahl gefundener Scan-Blöcke verwendet.