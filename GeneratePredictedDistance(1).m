function [Dpredict,D,Mtrain,SolveM,SolveN,DistanceTestErrorMean,DistanceTestErrorMax]=GeneratePredictedDistance(MLmodel,ssEN_dbm,txE,txN,UAVheightE,UAVheightN,UAVRcoor,sensorsiteE,sensorsiteN,sensorsiteNE,Height,sensornum)
   UAVheightE=elevation(txE)+UAVheightE;
   UAVheightN=elevation(txN)+UAVheightN;

for i=1:length(sensorsiteNE)
    for j=1:length(UAVRcoor)
    Rdistance(i,j)=((sensorsiteE(i)-UAVRcoor(j))^2+sensorsiteN(i)^2+(Height(i)-UAVheightE(j))^2)^(1/2);
    end
    
    for j=(length(UAVRcoor)+1):2*length(UAVRcoor)
    Rdistance(i,j)=((sensorsiteN(i)-UAVRcoor(j-length(UAVRcoor)))^2+sensorsiteE(i)^2+(Height(i)-UAVheightN(j-length(UAVRcoor)))^2)^(1/2);
    end
end
for i=1:length(ssEN_dbm(1,:))
    Sstrength(((i-1)*length(ssEN_dbm(:,1))+1):i*length(ssEN_dbm(:,1)))=ssEN_dbm(:,i);
end

for i=1:length(Rdistance(:,1))
    RdistanceR(((i-1)*length(Rdistance(1,:))+1):i*length(Rdistance(1,:)))=Rdistance(i,:);
end
Errortest=0;
Dpredict=predict(MLmodel,Sstrength');
RError=abs(RdistanceR'-Dpredict);
DistanceTestErrorMean=mean(RError);
DistanceTestErrorMax=max(RError);
for i=1:length(ssEN_dbm(1,:))
    D(i,:)=Dpredict(((i-1)*length(ssEN_dbm(:,1))+1):(i*length(ssEN_dbm(:,1))));%D is the prediction version of the matrix Rdistance
end
UAVcoorX=[UAVRcoor,zeros(1,length(UAVRcoor))];%construct new matrix of UAV coordinates for linera equation solving
UAVcoorY=[zeros(1,length(UAVRcoor)),UAVRcoor];%construct new matrix of UAV coordinates for linera equation solving
UAVcoorZ=[UAVheightE',UAVheightN'];%construct new matrix of UAV coordinates for linera equation solving

%----------construct matrix for solving linear equation-----------%
for i=1:length(D(:,1))
    for j=1:(floor(length(D(1,:))/2)-1)
        SolveM(i,j,1)=2*(UAVcoorX(j)-UAVcoorX(length(D(1,:))-j));%X corresponds to direction east
        SolveM(i,j,2)=2*(UAVcoorY(j)-UAVcoorY(length(D(1,:))-j));%y corresponds to direction north
        SolveM(i,j,3)=2*(UAVcoorZ(j)-UAVcoorZ(length(D(1,:))-j));
        SolveN(i,j)=D(i,length(D(1,:))-j)^2-D(i,j)^2-(UAVcoorX(length(D(1,:))-j)^2-UAVcoorX(j)^2)-(UAVcoorY(length(D(1,:))-j)^2-UAVcoorY(j)^2)-(UAVcoorZ(length(D(1,:))-j)^2-UAVcoorZ(j)^2);
    end
end
for i=1:length(D(:,1))
    M(i,:,1:3)=SolveM(i,:,:);
    M(i,:,4)=SolveN(i,:)';
end
for i=1:sensornum
    Mtrain(((1-1)*length(SolveN(1,:))+1):(1*length(SolveN(1,:))),i)=M(i,:,1);
    Mtrain(((1)*length(SolveN(1,:))+1):((1+1)*length(SolveN(1,:))),i)=M(i,:,2);
    Mtrain(((1+1)*length(SolveN(1,:))+1):((1+2)*length(SolveN(1,:))),i)=M(i,:,4);
    Mtrain(((1+2)*length(SolveN(1,:))+1):((1+3)*length(SolveN(1,:))),i)=M(i,:,4);
end

end
