function [coordsolved,Errormat1test]=PredictPositionLinear(Mtrain,tx,D,latr,lonr,SolveM,SolveN,sensorheight,transmittedpower,UAVRcoor,UAVheight,UAVheightE,UAVheightN,Height)
UAVcoorX=[UAVRcoor,zeros(1,length(UAVRcoor))];%construct new matrix of UAV coordinates for linera equation solving
UAVcoorY=[zeros(1,length(UAVRcoor)),UAVRcoor];%construct new matrix of UAV coordinates for linera equation solving
UAVcoorZ=[UAVheightE',UAVheightN'];%construct new matrix of UAV coordinates for linera equation solving
for i=1:length(D(:,1))
    M(:,:)=SolveM(i,:,:);
    coordsolved(i,1:3)=linsolve(M,SolveN(i,:)');
end

    
    
    
Dist=(coordsolved(:,1).^2+coordsolved(:,2).^2).^(1/2);
Angle=(atan(coordsolved(:,2)./coordsolved(:,1))*360)/(2*pi);
[latp,lonp]=location(tx,Dist,Angle);%get the sensors' position


            
            
            
            
rxp = rxsite('Name','Boston', ...
        'Latitude', latp, ...
        'Longitude',lonp, ...
         'ReceiverSensitivity', -90,...
         'AntennaHeight',sensorheight); %generate random position of sensor nodes within the designated area
H1=elevation(rxp);


sum=zeros(length(D(:,1)),1);
for i=1:length(D(:,1))
    for j=1:length(D(1,:))
        sum1(i,j)=D(i,j)^2-(coordsolved(i,1)-UAVcoorX(j))^2-(coordsolved(i,2)-UAVcoorY(j))^2;
        if (sum1(i,j)<=0)|(sum1(i,j)^(1/2)>UAVheight)
            sum(i)=sum(i)+H1(i);
        else
            sum(i)=sum(i)+UAVheight-sum1(i,j)^(1/2);
        
        end
    end
    sum(i)=sum(i)/length(D(1,:));
end


H2=sum-H1;
for i=1:length(H2)
    if H2(i)<0
        H2(i)=1;
    end
end
coordsolved(:,3)=H2+H1;

rxp = rxsite('Name','Boston', ...
        'Latitude', latp, ...
        'Longitude',lonp, ...
         'ReceiverSensitivity', -90,...
         'AntennaHeight',H2'); %generate random position of sensor nodes within the designated area

rxreal=txsite('Name','MathWorks', ...
        'Latitude', latr, ...
        'Longitude', lonr,...
        'AntennaHeight',sensorheight,'TransmitterPower',transmittedpower);%set one of the two UAV paths perpendicular to one another   
Errormattest=distance(rxp,rxreal);
for i=1:length(Errormattest(:,1))
    Errormat1test(i)=Errormattest(i,i);
end

 Errormat1test=(Errormat1test.^2+(coordsolved(:,3)'-Height').^2).^(1/2);
end