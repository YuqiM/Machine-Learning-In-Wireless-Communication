clear all vars
clc
close all
%%
%This provide a clearer view of the simulation platform. III. part can be customized
%Customize training by using different function, for instance at the
%position estimation part by using
%'GeneratePositionModelNet' -> 'PredictPositionNet' to estimate by neural
%network trained using 'fitnet' and 'trainlm'. 'GeneratePositionModelTree'
%-> 'PredictPositionTree' estimates the position using random forest
%Algorithm. By using 'PredictPositionLinear' solves the position using
%multilateration algorithm based on 'LS' method. The Output is Localization
%Error in meters for every sensor node. The Error estimated is produced
%using entirely test dataset. As a result the variable 'sensordensity'
%defines the test data size. 'sensordensitytrain' defines the size of the
%data set used to train the machine learning model. In II.part The distance is
%acquired by first call 'GenerateTrainedMLmodel' to train a RandomForest
%and then to call 'GeneratePredictedDistance' to get the predicted
%distance. 



Startla1=42.3001;%define the latitude at the start point of the UAV
Startlon1=-71.3503;%define the longitude at the start point of the UAV
UAVheight=120;%define UAV height above sea level
UAVspeed=3;%defined as meters per sencond
boxsize=1e3;%length in meters of the length of the square area
sensordensity=3000;%sensor density for localization defined as number of sensors per square kilometer
sensordensitytrain=500;
sensornum=round((sensordensity*boxsize^2)/1e6);
sensornum1=round((sensordensitytrain*boxsize^2)/1e6);
transmittedpower=10;%power of the transmitted power in watt
sensorheight=4;%sensors' distance above ground(meters)
receiversensitivity=-90;%receiver sensitivity in dBm
pm=propagationModel('longley-rice');%set the propagation modle to 'longlely-rice'.
UAVRcoor=[0:UAVspeed:(UAVspeed*floor(boxsize/UAVspeed))];
f = waitbar(0,'Loading your data');

%----------I.This part generate the raw data for post pocessing and in order to use the
%Terrain model internet connection is required and is often unstable.
[Height,ssEN_dbm,tx,txE,txN,txEN,UAVheightE,UAVheightN,sensorsiteE,sensorsiteN,sensorsiteNE,latr,lonr]=GenerateSimulationData(Startla1,Startlon1,UAVheight,UAVspeed,boxsize,sensordensity,...
sensornum,transmittedpower,sensorheight,receiversensitivity,pm);
[Height1,ssEN_dbm1,tx1,txE1,txN1,txEN1,UAVheightE1,UAVheightN1,sensorsiteE1,sensorsiteN1,sensorsiteNE1,latr1,lonr1]=GenerateSimulationData(Startla1,Startlon1,UAVheight,UAVspeed,boxsize,sensordensitytrain,...
sensornum1,transmittedpower,sensorheight,receiversensitivity,pm);
waitbar(.33,f,'Estimating Distance');

%----------II.This part estimates the distance based on trained model and RSS
[MLmodel,Dpredict1,D1,DistanceTrainErrorMean,DistanceTrainErrorMax]=GenerateTrainedMLmodel(ssEN_dbm1,UAVspeed,boxsize,sensorsiteE1,sensorsiteN1,sensorsiteNE1,Height1,txE1,txN1,UAVheightE1,UAVheightN1);
[Dpredict,D,Mtrain,SolveM,SolveN,DistanceTestErrorMean,DistanceTestErrorMax]=GeneratePredictedDistance(MLmodel,ssEN_dbm,txE,txN,UAVheightE,UAVheightN,UAVRcoor,sensorsiteE,sensorsiteN,sensorsiteNE,Height,sensornum);
waitbar(.66,f,'Estimating position');

%----------III.This part estimate the position of each sensor node

net= GeneratePositionModelNet(UAVRcoor,D1,UAVheightE1,UAVheightN1,sensornum1,sensorsiteE1,sensorsiteN1,Height1);
%[TreeX,TreeY,TreeZ]= GeneratePositionModelTree(UAVRcoor,D1,UAVheightE1,UAVheightN1,sensornum1,sensorsiteE1,sensorsiteN1,Height1);

[coordsolved,Errormat1test]=PredictPositionNet(net,Mtrain,tx,D,latr,lonr,sensorheight,transmittedpower,UAVRcoor,UAVheight,UAVheightE,UAVheightN,Height);
%[coordsolved,Errormat1test]=PredictPositionTree(TreeX,TreeY,TreeZ,Mtrain,tx,D,latr,lonr,sensorheight,transmittedpower,UAVRcoor,UAVheight,UAVheightE,UAVheightN,Height,sensornum);
%[coordsolved,Errormat1test]=PredictPositionLinear(Mtrain,tx,D,latr,lonr,SolveM,SolveN,sensorheight,transmittedpower,UAVRcoor,UAVheight,UAVheightE,UAVheightN,Height);
waitbar(1,f,'Finishing');
close(f)






























