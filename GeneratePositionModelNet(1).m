function net=GeneratePositionModelNet(UAVRcoor,D,UAVheightE,UAVheightN,sensornum,sensorsiteE,sensorsiteN,Height)
trainportion2=1;
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
for i=1:sensornum
    Mresult(i,1)=sensorsiteE(i);
    Mresult(i,2)=sensorsiteN(i);
    Mresult(i,3)=Height(i);
end


 
net=fitnet(20,'trainlm');
net.trainParam.max_fail=20;
net.trainParam.epochs=5000;
%net=configure(net,Mtrain(:,1:sensornum*trainportion2),Mresult(1:sensornum*trainportion2,:)');
 
%net=configure(net,Mtrain(:,1:sensornum*trainportion2),Mresult(1:sensornum*trainportion2,:)');

net=train(net,Mtrain(:,1:sensornum*trainportion2),Mresult(1:sensornum*trainportion2,:)');
end