%%
%%Author information
%Patrick(YuqiMeng) in CIS online program under professor Danijela Cabric 2019.10.17 version1.0
%%Brief description of its function
%This code perform the simulation of the transmission of the 16QAM signal
%first the random sequence of signal is generated then using the self
%defined function the signal is mapped to the constellation with only 1 bit
%difference between each adjacent point that approach is supposed to result
%in minimum BER then the signal is mapped to 2 quadrature 4ASK signal to
%create a 16QAM representation. The pulse shaping filter is designed using
%square root raised cosin transmitter and corresponding receiver. The
%spectrum of each stage of the signal transmission is shown in figure
%ploted into one figure in 'subplot' expression. The drawing of constellation
%require the signal have an imaginary part. So for convenience we send an
%addition signal using the complex representation at each tranmitter and
%receiver for mapping of constellation.And constellation of signal
%after receiver for all 4 types of channels are shown. And additionally
%constellation of the signal at some points of the system before receiver
%is shown also as it is in the modulated form its spread in the graph can be considerable
%BER: BER is calculated using the self defined function 'BERestimate' after
%the signal have gone through many stages and passes many filters.
%Missynchronization can occur and the received signal can suffer an
%additional delay. So this function comes useful in the BER
%evalution. Further information such as input and output is described
%inside the function.
%The ouput BER shown is the BER after 15dB awgn channel, for more BER
%estimate specific input of corresponding functions can be adjusted to
%calculate theirs.
%The constellation mapping method used here is for [1 j]->1100,[3
%j]->1101,[1 j*3]->1110,[3 j*3]->1111,[1 j*-1]->0100,[3 j*-1]->0110,[1
%j*-3]->0101,[3 j*-3]=0111,[-1 j]=1000,[-1 3j]->1001,[-3 j]->1010,[-3
%3j]->1011,[-1 -j]->0000,[-1 -3j]->0010,[-3 -j]->0001,[-3 -3j]->0011
%About adaptability:
%This code is written in matlabR2016a and for convenience some functions
%which are not user defined is used which includes functions like'ammod'
%which result in the data be amplitude modulated. And 'amdemod' to
%demodulate the AM signal. And constellation generation uses the function
%'comm.ConstellationDiagram' which is included in the communication
%toolbox. And some signal mapping methods such as comm.PAMModulator is also
%used.
%%Main variable
%O_data:generate original data
%N_sample£ºsample rate
%BW: bandwidth of the transmitted signal
%fc: carrier frequency to mudulate on.
%fs:symbol rate
%%code


clear all vars
clc
BW=20e6;%bandwidth set to 20MHz
fs=BW/2;%corresponding symbol rate
M=16;
k=log2(M);
bitrate=fs*k;%the corresponding bitrate
N_sample=16;%sample rate
O_data=randi([0 M-1],4000,1);%generate data for transmission
RollOffFactor=1;%define the raised cosin square root pulse shaping filter to specify its Rollofffactor to be '1'
fc=70e6;%carrier frequency for analog modulation specified to be 70MHz
cd = comm.ConstellationDiagram('XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the original signal');%specify the constellation object for draiwng of constellation each with specific title
cd1 = comm.ConstellationDiagram('XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the modulated signal for transmission');
cd2 = comm.ConstellationDiagram('XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the modulated signal after 15dB AWGN channel');
cd3 = comm.ConstellationDiagram('XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the modulated signal after 5dB AWGN channel');
cd4 = comm.ConstellationDiagram('XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the modulated signal after case 1 multipath channel');
cd5 = comm.ConstellationDiagram('XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the modulated signal after case 2 multipath channel');
cd6 = comm.ConstellationDiagram('XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the received signal after 15dB AWGN channel');
cd7 = comm.ConstellationDiagram('XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the received signal after 5dB AWGN channel');
cd8 = comm.ConstellationDiagram('XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the received signal after case 1 multipath channel');
cd9 = comm.ConstellationDiagram('XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the received signal after case 2 multipath channel');

%-------map the data to two 4ASK signal-------%
[a,b]=qammap16(O_data);%a is assumed to be the real part of the divided 4ASK scheme b is assumed to be the imaginary part
%--------square root pulse shape-------------%
Txraise=comm.RaisedCosineTransmitFilter('Shape','Square root','RolloffFactor',...
    RollOffFactor,'OutputSamplesPerSymbol',N_sample,'Gain',1,'FilterSpanInSymbols',1);%generate the raised cosin filter object
a=a';%convert the inphase signal to row vector representation for the purpose of successfully implement the pulse shaping filter
b=b';%convert the Quadrature signal to row vector representation for the purpose of successfully implement the pulse shaping filter

sumori=a+j*b;%produce an additional complex representation of the signal for the drawing of constellation
step(cd,sumori);%pass the complex representaion of the signal through pulse shaping filter 
atran=step(Txraise,a);%pass the real inphase representaion of the signal through pulse shaping filter 
btran=step(Txraise,b);%pass the real quadrature representaion of the signal through pulse shaping filter 
[Fatran,Patran]=tospec(atran,fs,N_sample);%generate the data for spectrum graph
[Fbtran,Pbtran]=tospec(btran,fs,N_sample);%generate the data for spectrum graph

%------------modulate the signal onto the carrier----------%
Amod=ammod(atran,fc,fs*N_sample);%construct passband signal
Bmod=ammod(btran,fc,fs*N_sample,pi/2);%use the quadrature carrier with 'initial phase' to be pi/2 

Summod=Amod+Bmod;%the signal is then combined for transmission

Sumconst=Amod+j*Bmod;%the corresponding combined complex signal for the purpose of constellation drawing

step(cd1,Sumconst);%view the constellation of the modulated complex representation of the transmitted signal  
[Fsum,Psum]=tospec(Summod,fs,N_sample);%generate the data for spectrum graph
%------------pass the signal through channel--------------%
chanawgn15=comm.AWGNChannel('NoiseMethod','Signal to noise ratio (SNR)','SNR',15);%create the AWGN channel object with specified SNR
chanawgn5=comm.AWGNChannel('NoiseMethod','Signal to noise ratio (SNR)','SNR',5);
Multichan1=comm.RayleighChannel('SampleRate',N_sample,'PathDelays',[0 30e-9],'AveragePathGains',[0 -3]);%set the relative path delays to [0 30ns] relative power to [0 -3dB]
Multichan2=comm.RayleighChannel('SampleRate',N_sample,'PathDelays',[0 1e-9],'AveragePathGains',[0 -3]);%set the relative path delays to [0 1ns] relative power to [0 -3dB]

Txdata15dB=step(chanawgn15,Summod);%pass the combined real signal through awgn channel with 15dB SNR
Txdata5dB=step(chanawgn5,Summod);%pass the combined real signal through awgn channel with 5dB SNR
TxdataMP1=step(Multichan1,Summod);%pass the signal through multipath channel case 1
TxdataMP2=step(Multichan2,Summod);%pass the signal through multipath channel case 2

release(chanawgn15);%for the channel to adapt to the complex representation of the signal 'release' the channel object first
Txconst15=step(chanawgn15,Sumconst);%pass the combined complex signal through awgn channel with 15dB SNR
release(chanawgn5);%for the channel to adapt to the complex representation of the signal 'release' the channel object first
Txconst5=step(chanawgn5,Sumconst);%pass the combined complex signal through awgn channel with 5dB SNR
release(Multichan1);%for the channel to adapt to the complex representation of the signal 'release' the channel object first
TxconstM1=step(Multichan1,Sumconst);%pass the complex signal through multipath channel case 1
release(Multichan2);%for the channel to adapt to the complex representation of the signal 'release' the channel object first
TxconstM2=step(Multichan2,Sumconst);%pass the complex signal through multipath channel case 2

step(cd2,Txconst15);%view the constellation of the complex signal which has passed 15dB awgn channel
step(cd3,Txconst5);%view the constellation of the complex signal which has passed 5dB awgn channel
step(cd4,TxconstM1);%view the constellation of the complex signal which has passed case 1 multipath channel
step(cd5,TxconstM2);%view the constellation of the complex signal which has passed case 2 multipath channel



[F15,P15]=tospec(Txdata15dB,fs,N_sample);%generate the data necessary for the drawing of spectrum
[F5,P5]=tospec(Txdata5dB,fs,N_sample);
[FM1,PM1]=tospec(TxdataMP1,fs,N_sample);
[FM2,PM2]=tospec(TxdataMP2,fs,N_sample);

%-------receive the signal------------%
RxdataA15=amdemod(Txdata15dB,fc,fs*N_sample);%demodulate the Inphase signal of the 15dB awgn channel
RxdataB15=amdemod(Txdata15dB,fc,fs*N_sample,pi/2);%demodulate the quadrature signal of the 15dB awgn channel
RxdataA5=amdemod(Txdata5dB,fc,fs*N_sample);%demodulate the Inphase signal of the 5dB awgn channel
RxdataB5=amdemod(Txdata5dB,fc,fs*N_sample,pi/2);%demodulate the quadrature signal of the 5dB awgn channel

RxdataMPA1=amdemod(real(TxdataMP1)+imag(TxdataMP1),fc,fs*N_sample);%demodulate the Inphase signal of the case 1 multipath channel
RxdataMPB1=amdemod(real(TxdataMP1)+imag(TxdataMP1),fc,fs*N_sample,pi/2);%demodulate the quadrature signal of the case 2 multipath channel
RxdataMPA2=amdemod(real(TxdataMP2)+imag(TxdataMP2),fc,fs*N_sample);%demodulate the Inphase signal of the case 2 multipath channel
RxdataMPB2=amdemod(real(TxdataMP2)+imag(TxdataMP2),fc,fs*N_sample,pi/2);%demodulate the quadrature signal of the case 2 multipath channel

RxdataMPAconst1=amdemod(real(TxconstM1),fc,fs*N_sample);%demodulate the complex representation of the inphase signal for the case 1 multipath channel
RxdataMPBconst1=amdemod(imag(TxconstM1),fc,fs*N_sample,pi/2);%demodulate the complex representation of the quadrature signal for the case 1 multipath channel
RxdataMPAconst2=amdemod(real(TxconstM2),fc,fs*N_sample);%demodulate the complex representation of the inphase signal for the case 2 multipath channel
RxdataMPBconst2=amdemod(imag(TxconstM2),fc,fs*N_sample,pi/2);%demodulate the complex representation of the quadrature signal for the case 2 multipath channel



[FRA15,PRA15]=tospec(RxdataA15,fs,N_sample);%generate the data necessary for the drawing of spectrum
[FRB15,PRB15]=tospec(RxdataB15,fs,N_sample);
[FRA5,PRA5]=tospec(RxdataA5,fs,N_sample);
[FRB5,PRB5]=tospec(RxdataB5,fs,N_sample);
[FRAC1,PRAC1]=tospec(RxdataMPA1,fs,N_sample);
[FRBC1,PRBC1]=tospec(RxdataMPB1,fs,N_sample);
[FRAC2,PRAC2]=tospec(RxdataMPA2,fs,N_sample);
[FRBC2,PRBC2]=tospec(RxdataMPB2,fs,N_sample);

%-------pass the signal through squared root raised cosin receiver------%
Rxraise=comm.RaisedCosineReceiveFilter('Shape',...
    'Square root','RolloffFactor',RollOffFactor,'InputSamplesPerSymbol',N_sample,'Gain',1,'DecimationFactor',N_sample,'FilterSpanInSymbols',1);
RxdataA151=step(Rxraise,RxdataA15);%pass the received inphase signal through raised cosin square root filter receiver for the 15dB awgn channel case
RxdataB151=step(Rxraise,RxdataB15);%pass the received quadrature signal through raised cosin square root filter receiver for the 15dB awgn channel case
RxdataA51=step(Rxraise,RxdataA5);%pass the received inphase signal through raised cosin square root filter receiver for the 5dB awgn channel case
RxdataB51=step(Rxraise,RxdataB5);%pass the received quadrature signal through raised cosin square root filter receiver for the 5dB awgn channel case

RxdataAM1=step(Rxraise,RxdataMPA1);%pass the received inphase signal through raised cosin square root filter receiver for the case 1 multipath channel
RxdataBM1=step(Rxraise,RxdataMPB1);%pass the received quadrature signal through raised cosin square root filter receiver for the case 1 multipath channel
RxdataAM2=step(Rxraise,RxdataMPA2);%pass the received inphase signal through raised cosin square root filter receiver for the case 2 multipath channel
RxdataBM2=step(Rxraise,RxdataMPB2);%pass the received quadrature signal through raised cosin square root filter receiver for the case 2 multipath channel

release(Rxraise);% for implementing receiver of the complex signal use 'release'
RxconstMDA1=step(Rxraise,RxdataMPAconst1+j*RxdataMPBconst1);%for the drawing of the constellation we also pass the demodulated dcomplex representation of the signal for case 1 of the multipath channel
RxconstMDB1=step(Rxraise,RxdataMPBconst1);% we also pass the demodulated quadrature part complex representation of the signal for case 1 of the multipath channel
RxconstMDA2=step(Rxraise,RxdataMPAconst2+j*RxdataMPBconst2);%for the drawing of the constellation we also pass the demodulated dcomplex representation of the signal for case 2 of the multipath channel
RxconstMDB2=step(Rxraise,RxdataMPBconst2);%we also pass the demodulated quadrature part complex representation of the signal for case 2 of the multipath channel


step(cd6,RxdataA151+j*RxdataB151);%draw constellation of the signal at the receiver through 15dB awgn channel
step(cd7,RxdataA51+j*RxdataB51);%draw constellation of the signal at the receiver through 5dB awgn channel
step(cd8,RxconstMDA1);%draw constellation of the signal at the receiver through case 1 multipath channel
step(cd9,RxconstMDA2);%draw constellation of the signal at the receiver through case2 multipath channel




mod1=comm.PAMDemodulator;%create a demodulation object for convenient determination of the received signal
mod2 = comm.PAMModulator('ModulationOrder',4);%specify the object

RxdataA152=real(step(mod2,step(mod1,RxdataA151)));%get the received Inphase signal for the 15dB awgn channel case
RxdataB152=real(step(mod2,step(mod1,RxdataB151)));%get the received quadrature signal for the 15dB awgn channel case
RxdataA52=real(step(mod2,step(mod1,RxdataA51)));%get the received Inphase signal for the case 1 multipath channel
RxdataB52=real(step(mod2,step(mod1,RxdataB51)));%get the received quadrature signal for the case 2 multipath channel
%-----recombine the signal and calculate the BER--------%
bitsent=qam16tobit(O_data);%convert the decimal representation of the original data to binary for BER estimation
bitreceive=qam16recombine(RxdataA152,RxdataB152);%perform the parallel to serial transform and map the signal to binary
[Errcount,BER,Missync]=BERestimate(bitreceive,bitsent)%here the bit error rate for the 15dB awgn channel and 5dB awgn channel is calculated and by tuning the input of the function 
%'qam16recombine' the BER of either channel can be shown. This will require
%corresponding function to be loaded.

%-------generate the corresponging figures------------%
h9=figure(1)
subplot(5,3,1)
plot(Fatran,Patran);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of Inphase 4ASK signal after pulse shaping');

subplot(5,3,2)
plot(Fbtran,Pbtran);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of Quadrature 4ASK signal after pulse shaping');

subplot(5,3,3)
plot(Fsum,Psum);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of modulated signal');

subplot(5,3,4)
plot(F15,P15);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of signal after 15dB AWGN channel');

subplot(5,3,5)
plot(F5,P5);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of signal after 5dB AWGN channel');

subplot(5,3,6)
plot(FM1,PM1);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of signal after Multipath case 1 channel');

subplot(5,3,7)
plot(FM2,PM2);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of signal after Multipath case 2 channel');

subplot(5,3,8)
plot(FRA15,PRA15);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Inphase signal after demodulation for 15dB AWGN case');

subplot(5,3,9)
plot(FRB15,PRB15);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Quadrature signal after demodulation for 15dB AWGN case');

subplot(5,3,10)
plot(FRA5,PRA5);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Inphase signal after demodulation for 5dB AWGN case');

subplot(5,3,11)
plot(FRB5,PRB5);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Quadrature signal after demodulation for 5dB AWGN case');
subplot(5,3,12)
plot(FRAC1,PRAC1);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Inphase signal after demodulation for case 1 multipath channel');

subplot(5,3,13)
plot(FRBC1,PRBC1);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Quadrature signal after demodulation for case 1 multipath channel');

subplot(5,3,14)
plot(FRAC2,PRAC2);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Inphase signal after demodulation for case 2 multipath channel');

subplot(5,3,15)
plot(FRBC2,PRBC2);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Quadrature signal after demodulation for case 2 multipath channel');


h1=eyediagram(RxdataA151,2);
title('eyediagram for the received Inphase signal in the 15dB AWGN channel case');
%print(h1,'G:\CIS-research\HW1\simulation results\16QAM\eyediagram for the received Inphase signal in the 15dB AWGN channel case.jpeg','-djpeg','-r300')

h2=eyediagram(RxdataB151,2);
title('eyediagram for the received Quadrature signal in the 15dB AWGN channel case');
%print(h2,'G:\CIS-research\HW1\simulation results\16QAM\eyediagram for the received Quadrature signal in the 15dB AWGN channel case.jpeg','-djpeg','-r300')

h3=eyediagram(RxdataA51,2);
title('eyediagram for the received Inphase signal in the 5dB AWGN channel case');
%print(h3,'G:\CIS-research\HW1\simulation results\16QAM\eyediagram for the received Inphase signal in the 5dB AWGN channel case.jpeg','-djpeg','-r300')

h4=eyediagram(RxdataB51,2);
title('eyediagram for the received Quadrature signal in the 5dB AWGN channel case');
%print(h4,'G:\CIS-research\HW1\simulation results\16QAM\eyediagram for the received Quadrature signal in the 5dB AWGN channel case.jpeg','-djpeg','-r300')

h5=eyediagram(RxdataAM1,2);
title('eyediagram for the received Inphase signal in the case 1 multichannel');
%print(h5,'G:\CIS-research\HW1\simulation results\16QAM\eyediagram for the received Inphase signal in the case 1 multichannel.jpeg','-djpeg','-r300')

h6=eyediagram(RxdataBM1,2);
title('eyediagram for the received Quadrature signal in the case 1 multichannel');
%print(h6,'G:\CIS-research\HW1\simulation results\16QAM\eyediagram for the received Quadrature signal in the case 1 multichannel.jpeg','-djpeg','-r300')

h7=eyediagram(RxdataAM2,2);
title('eyediagram for the received Inphase signal in the case 2 multichannel');
%print(h7,'G:\CIS-research\HW1\simulation results\16QAM\eyediagram for the received Inphase signal in the case 2 multichannel.jpeg','-djpeg','-r300')

h8=eyediagram(RxdataBM2,2);
title('eyediagram for the received Quadrature signal in the case 2 multichannel');
%print(h8,'G:\CIS-research\HW1\simulation results\16QAM\eyediagram for the received Quadrature signal in the case 2 multichannel.jpeg','-djpeg','-r300')












        
        
        
        
        
        
        
        
        
        
        

