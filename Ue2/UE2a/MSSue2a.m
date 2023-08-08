%Prozessierung der ROSbag-Daten zu einer .mat-Datei für die weitere Auswertung
%% Einlesen der ROSbag-Datei und Speicherung der IMU-Messungen in eine .mat-Datei
% In diesem Abschnitt soll die ROSbag-Datei eingelesen und gespeichert werden. Der genaue Ablauf ist im Code-Template aufgezeigt.
% Der Name des ROSbag ist:         
%                            2021-11-15-10-28-58.bag

% Der Datensatz kann heruntergeladen und lokal verarbeitet werden.

% Die Feldnamen der Eingangs- und Ausgangsvariablen sollten wie folgt lauten:
% Input:
%                    bag.Rosbag
%                    bag.imutopic
% Output:
%                    imu_data.startTime
%                    imu_data.angularvelocity
%                    imu_data.linearacceleration
%                    imu_data.rostime

%Initialisieren der Inputparameter:
% - bag.Rosbag (Einlesen der Rosbag-Datei):

bag.Rosbag =  rosbag('2021-11-15-10-28-58.bag') ;

% - bag.imutopic (Setzen der richtigen Rostopic):

%  char 将数字转换为字符
bag.imutopic = char(bag.Rosbag.MessageList.Topic(1)); 
%data = readMessages(bag.Rosbag,'DataFormat','struct');
%bag.imutopic = select(bag.Rosbag, "Topic","/vectornav/IMU"); 

%imu = readMessages(imu_msg)

%Aufrufen der Ladefunktion der imu_data-Variable:
imu_data = read_imudata_from_rosbag(bag);
%display(imu_data)

%Darstellen der Linearen Beschleunigung durch Plot(s): 
% - (wird im Nachhinein noch einmal händisch geprüft)

figure
hold on
x = imu_data.linearacceleration(:,1);
y = imu_data.linearacceleration(:,2);
z = imu_data.linearacceleration(:,3);
plot(imu_data.rostime,x);
plot(imu_data.rostime,y);
plot(imu_data.rostime,z);
xlabel('rostime (Sec)')
ylabel('Acceleration (M/s^2)')
legend('acc\_x','acc\_y','acc\_z')
grid on

%% Umspeicherung für das feinere MatlabGrader-Überprüfungsfeedback
bag_Rosbag =  bag.Rosbag
bag_imutopic = bag.imutopic
imu_data_startTime = imu_data.startTime
imu_data_angularvelocity = imu_data.angularvelocity
imu_data_linearacceleration = imu_data.linearacceleration
imu_data_rostime = imu_data.rostime



%Ladefunktion:
function imu_data=read_imudata_from_rosbag(bag)
    %bag.imutopic = select(bag.Rosbag, "Topic","/vectornav/IMU");
    imu_msg = select(bag.imutopic,'MessageType','sensor_msgs/Imu');
    imu_all = readMessages(imu_msg);
    M_av = [];
    M_la = [];
    M_rt = [];
    for i = 1:imu_msg.NumMessages
        imu = imu_all(i);
        vel_x = imu{1}.AngularVelocity.X;
        vel_y = imu{1}.AngularVelocity.Y;
        vel_z = imu{1}.AngularVelocity.Z;
        M_av = [M_av;[vel_x vel_y vel_z]];

        acc_x = imu{1}.LinearAcceleration.X;
        acc_y = imu{1}.LinearAcceleration.Y;
        acc_z = imu{1}.LinearAcceleration.Z; 
        M_la = [M_la;[acc_x acc_y acc_z]];  
    end

    imu_data.startTime = bag.imutopic.StartTime;
    imu_data.rostime = table2array(bag.imutopic.MessageList(:,1)); 
    imu_data.angularvelocity =  M_av;
    imu_data.linearacceleration =  M_la; 

end