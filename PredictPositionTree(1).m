function [coordsolved,Errormat1test]=PredictPositionTree(TreemodelR1,TreemodelR2,TreemodelR3,Mtrain,tx,D,latr,lonr,sensorheight,transmittedpower,UAVRcoor,UAVheight,UAVheightE,UAVheightN,Height,sensornum)
UAVcoorX=[UAVRcoor,zeros(1,length(UAVRcoor))];%construct new matrix of UAV coordinates for linera equation solving
UAVcoorY=[zeros(1,length(UAVRcoor)),UAVRcoor];%construct new matrix of UAV coordinates for linera equation solving
UAVcoorZ=[UAVheightE',UAVheightN'];%construct new matrix of UAV coordinates for linera equation solving

tic
coordsolved1=predict(TreemodelR1, Mtrain(:,1:sensornum)');
coordsolved2=predict(TreemodelR2, Mtrain(:,1:sensornum)');
coordsolved3=predict(TreemodelR3, Mtrain(:,1:sensornum)');
toc
coordsolved=[coordsolved1,coordsolved2,coordsolved3];
Dist=(coordsolved(:,1).^2+coordsolved(:,2).^2).^(1/2);
Angle=(atan(coordsolved(:,2)./coordsolved(:,1))*360)/(2*pi);
[latp,lonp]=location(tx,Dist,Angle);%get the sensors' position


            
            
            
            
rxp = rxsite('Name','Boston', ...
        'Latitude', latp, ...
        'Longitude',lonp, ...
         'ReceiverSensitivity', -90,...
         'AntennaHeight',sensorheight); %generate random position of sensor nodes within the designated area
H1=elevation(rxp);
H2=coordsolved3-H1;
for i=1:length(H2)
    if H2(i)<0
        H2(i)=1;
    end
end
coordsolved3=H2+H1;
rxp = rxsite('Name','Boston', ...
        'Latitude', latp, ...
        'Longitude',lonp, ...
         'ReceiverSensitivity', -90,...
         'AntennaHeight',H2'); %generate random position of sensor nodes within the designated area

rxreal=txsite('Name','MathWorks', ...
        'Latitude', latr, ...
        'Longitude', lonr,...
        'AntennaHeight',sensorheight,'TransmitterPower',transmittedpower);%set one of the two UAV paths perpendicular to one another   
Errormattest=distance(rxp,rxreal,'euclidean');
for i=1:length(Errormattest(:,1))
    Errormat1test(i)=Errormattest(i,i);
end

 Errormat1test=(Errormat1test.^2+(coordsolved3'-Height').^2).^(1/2);
end