%3D-Rotationsmatrizen definieren
% Bevor die Transformationen in die unterschiedlichen Koordinatensysteme durchgeführt werden kann, muss vorab die Funktion 
% rotmat.m geschrieben werden. Diese berechnet in Abhängigkeit der Eingabeparameter: Rotationsachse - 'axis' und 
% Drehwinkel 'angle' die entsprechende 3D-Rotationsmatrix. Für die zeitlich stabile Transformation wird die Rotationsmatrix 
% nur einmalig aufgestellt, die Berücksichtigung der zeitlich variablen Transformationen vom TKS zum LKS bedürfen jedoch 
% eine Vielzahl an Rotationsmatrizen, sodass es dieser Funktion dafür benötigt.
% Die Übergabeparameter an die Funktion sind die Rotationsachse  'axis' --> 'x', 'y' oder 'z' <-- (als char) sowie der Drehwinkel
% 'angle' --> , oder  <-- (als double in [rad]).
% Der Rückgabewert ist ausschließlich die 3x3 Rotationsmatrix R, wobei die unten angegebene Definition gilt.
% Es muss beachtet werden, dass die Funktion insoweit variable sein muss, dass je nach gewählter Rotationsachse 
% die Aufstellung der Rotationsmatrix erfolgt.
% Hinweis: die fertige Version dieser Funktion wird in den nachfolgenden Teilen benötigt und kann entsprechend kopiert werden!

omega=pi;
phi=pi/2;
kappa=pi/4;
rotmat('x', omega);
rotmat('y', phi);
rotmat('z', kappa);

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
