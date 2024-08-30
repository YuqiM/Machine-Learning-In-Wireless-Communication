function [MLmodel,Dpredict,D,ErrortrainMean,ErrortrainMax]=GenerateTrainedMLmodel(ssEN_dbm,UAVspeed,boxsize,sensorsiteE,sensorsiteN,sensorsiteNE,Height,txE,txN,UAVheightE,UAVheightN)
trainportion=1;
UAVRcoor=[0:UAVspeed:(UAVspeed*floor(boxsize/UAVspeed))];
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
MLmodel=TreeBagger(50,Sstrength(1:length(Sstrength)*trainportion)',RdistanceR(1:length(RdistanceR)*trainportion)','Method','regression','MinLeafSize',5);
Errortrain=0;
Dpredict=predict(MLmodel,Sstrength');
for i=1:length(ssEN_dbm(1,:))
    D(i,:)=Dpredict(((i-1)*length(ssEN_dbm(:,1))+1):(i*length(ssEN_dbm(:,1))));%D is the prediction version of the matrix Rdistance
end
RError=abs(RdistanceR'-Dpredict);
ErrortrainMean=mean(RError);
ErrortrainMax=max(RError);
end