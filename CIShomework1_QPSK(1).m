%%
%%Author information
%Patrick(YuqiMeng) in CIS online program under professor Danijela Cabric 2019.10.17 version1.0
%%Brief description of its function
%This code perform the simulation of the transmission of the QPSK signal
%first the random sequence of signal is generated then using the self
%defined function the signal is mapped to the constellation with only 1 bit
%difference between each adjacent point that approach is supposed to result
%in minimum BER then the signal is mapped to 2 quadrature 2ASK signal to
%create a QPSK representation. The pulse shaping filter is designed using
%square root raised cosin for transmitter and corresponding receiver. The
%spectrum of each stage of the signal transmission is shown in figure
%ploted into one figure in 'subplot' expression.The drawing of constellation
%require the signal have an imaginary part. So for convenience we send an
%addition signal using the complex representation at each tranmitter and
%receiver for mapping of constellation.
%And constellation of signal after receiver for all 4 types of channels are shown. 
%Additionally constellation of the signal at some points of the system before receiver
%is also shown as it is in the modulated form its spread in the graph can be considerable
%BER: BER is calculated using the self defined function 'BERestimate' after
%the signal have gone through many stages and passes many filters.
%Missynchronization can occur and the received signal can suffer an
%additional delay. So this function comes useful in the BER
%evalution. Further information such as input and output is described
%inside the function.
%The ouput BER shown is the BER after 15dB awgn channel, for more BER
%estimate specific input of corresponding functions can be adjusted to
%calculate theirs.
%
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
BW=40e6;%bandwidth of the passband signal
fs=BW/2;%corresponding symbol rate
M=4;%modulation order
k=log2(M);%number of bits each symbol represents
bitrate=fs*k;%transmitted bitrate
N_sample=16;%sample rate
O_data=randi([0 M-1],4000,1);%generate random information for transmission
RollOffFactor=1;
fc=1e8;
mod = comm.QPSKModulator;

cd1 = comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the modulated signal');%specify the constellation object for draiwng of constellation each with specific title
cd2 = comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the signal after 15dB AWGN channel');
cd3 = comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the signal after demodulation');
cd4 = comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the signal after receiver for 15dB AWGN channel');
cd5 = comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the signal after 5dB AWGN channel');
cd6 = comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the signal after receiver for 5dB AWGN channel');
cd7=comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the received signal of Multipath 1 channel');
cd8=comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the received signal of Multipath 2 channel');



for i=1:length(O_data)%this step maps the signal to two parallel used to create 2 2ASK signal with quadrature phase for the generation of QPSK signal
    if O_data(i)==0;
        a(i)=-1;
        b(i)=-1;
    elseif O_data(i)==1;
        a(i)=1;
        b(i)=-1;
    elseif O_data(i)==2;
        a(i)=-1;
        b(i)=1;
    else O_data(i)==3;
        a(i)=1;
        b(i)=1;
    end
end
Txraise=comm.RaisedCosineTransmitFilter('Shape','Square root','RolloffFactor',...
    RollOffFactor,'OutputSamplesPerSymbol',N_sample,'Gain',1,'FilterSpanInSymbols',1);%generate the raised cosin filter object
a=a';%convert the inphase signal to row vector representation for the purpose of successfully implement the pulse shaping filter
b=b';%convert the Quadrature signal to row vector representation for the purpose of successfully implement the pulse shaping filter
atran=step(Txraise,a);%pass the real inphase representaion of the signal through pulse shaping filter 
btran=step(Txraise,b);%pass the real quadrature representaion of the signal through pulse shaping filter 




%-----annlog passband modulation-------%
Amod=ammod(atran,fc,fs*N_sample);%construct passband signal
Bmod=ammod(btran,fc,fs*N_sample,pi/2);%use the quadrature carrier with 'initial phase' to be pi/2 


Summod=Amod+Bmod;%generate the signal for transmission
Summodconst=Amod+j*Bmod;%the corresponding combined complex signal for the purpose of constellation drawing
step(cd1,Summodconst)%view the constellation of the modulated complex representation of the transmitted signal  
[FT,PT]=tospec(Summod,fs,N_sample);%generate the data for spectrum graph
%-----pass signal through channel------%
chanawgn15=comm.AWGNChannel('NoiseMethod','Signal to noise ratio (SNR)','SNR',15);%create the AWGN channel object with specified SNR
chanawgn5=comm.AWGNChannel('NoiseMethod','Signal to noise ratio (SNR)','SNR',5);
Multichan1=comm.RayleighChannel('SampleRate',N_sample,'PathDelays',[0 30e-9],'AveragePathGains',[0 -3]);%set the relative path delays to [0 30ns] relative power to [0 -3dB]
Multichan2=comm.RayleighChannel('SampleRate',N_sample,'PathDelays',[0 1e-9],'AveragePathGains',[0 -3]);%set the relative path delays to [0 1ns] relative power to [0 -3dB]

Txdata15dB=step(chanawgn15,Summod);%pass the combined real signal through awgn channel with 15dB SNR
Txdata5dB=step(chanawgn5,Summod);%pass the combined real signal through awgn channel with 5dB SNR
TxdataMP1=step(Multichan1,Summod);%pass the signal through multipath channel case 1
TxdataMP2=step(Multichan2,Summod);%pass the signal through multipath channel case 2
release(chanawgn15)%for the channel to adapt to the complex representation of the signal 'release' the channel object first
Summodconst15=step(chanawgn15,Summodconst);%pass the combined complex signal through awgn channel with 15dB SNR
step(cd2,Summodconst15);%view the constellation of the complex signal which has passed 15dB awgn channel
release(chanawgn5);%for the channel to adapt to the complex representation of the signal 'release' the channel object first
Summodconst5=step(chanawgn5,Summodconst);%pass the combined complex signal through awgn channel with 5dB SNR
step(cd5,Summodconst5);%view the constellation of the complex signal which has passed 5dB awgn channel

release(Multichan1)%for the channel to adapt to the complex representation of the signal 'release' the channel object first
SummodconstC1=step(Multichan1,Summodconst);%pass the complex signal through multipath channel case 1
release(Multichan2);%for the channel to adapt to the complex representation of the signal 'release' the channel object first
SummodconstC2=step(Multichan2,Summodconst);%pass the complex signal through multipath channel case 2





[F15,P15]=tospec(Txdata15dB,fs,N_sample);
[F5,P5]=tospec(Txdata5dB,fs,N_sample);
[FM1,PM1]=tospec(TxdataMP1,fs,N_sample);
[FM2,PM2]=tospec(TxdataMP2,fs,N_sample);

%-------receive the signal------------%
RxdataA15=amdemod(Txdata15dB,fc,fs*N_sample);%demodulate the Inphase signal of the 15dB awgn channel
RxdataB15=amdemod(Txdata15dB,fc,fs*N_sample,pi/2);%demodulate the quadrature signal of the 15dB awgn channel
RxdataA5=amdemod(Txdata5dB,fc,fs*N_sample);%demodulate the Inphase signal of the 5dB awgn channel
RxdataB5=amdemod(Txdata5dB,fc,fs*N_sample,pi/2);%demodulate the quadrature signal of the 5dB awgn channel

RxdataAC1=amdemod(real(TxdataMP1)+imag(TxdataMP1),fc,fs*N_sample);%demodulate the Inphase signal of the case 1 multipath channel
RxdataBC2=amdemod(real(TxdataMP1)+imag(TxdataMP1),fc,fs*N_sample,pi/2);%demodulate the quadrature signal of the case 2 multipath channel
RxdataAC2=amdemod(real(TxdataMP2)+imag(TxdataMP2),fc,fs*N_sample);%demodulate the Inphase signal of the case 2 multipath channel
RxdataBC2=amdemod(real(TxdataMP2)+imag(TxdataMP2),fc,fs*N_sample,pi/2);%demodulate the quadrature signal of the case 2 multipath channel


RxconstR15=amdemod(real(Summodconst15),fc,fs*N_sample);%a complex representation of the signal used to draw constellation
RxconstI15=amdemod(imag(Summodconst15),fc,fs*N_sample,pi/2);
de_const15=RxconstR15+j*RxconstI15;%combine the inphase and quadrature signal for complex representation
step(cd3,de_const15);%draw constellation for 15dB awgn channel
RxconstR5=amdemod(real(Summodconst5),fc,fs*N_sample);%demodulate for the inphase part of the signal for 5dB awgn channel
RxconstI5=amdemod(imag(Summodconst5),fc,fs*N_sample,pi/2);%demodulate for the quadrature part of the signal
de_const5=RxconstR5+j*RxconstI5;%combine the inphase and quadrature signal for complex representation

RxconstRC1=amdemod(real(SummodconstC1),fc,fs*N_sample);%demodulate for the inphase part of the signal for case 1 multichannel
RxconstIC1=amdemod(imag(SummodconstC1),fc,fs*N_sample,pi/2);%demodulate for the quadrature part of the signal
de_constC1=RxconstRC1+j*RxconstIC1;%combine the inphase and quadrature signal for complex representation

RxconstRC2=amdemod(real(SummodconstC2),fc,fs*N_sample);%demodulate for the inphase part of the signal for case 2 multichannel
RxconstIC2=amdemod(imag(SummodconstC2),fc,fs*N_sample,pi/2);%demodulate for the quadrature part of the signal
de_constC2=RxconstRC2+j*RxconstIC2;%combine the inphase and quadrature signal for complex representation

[FRA15,PRA15]=tospec(RxdataA15,fs,N_sample);
[FRB15,PRB15]=tospec(RxdataB15,fs,N_sample);
[FRA5,PRA5]=tospec(RxdataA5,fs,N_sample);
[FRB5,PRB5]=tospec(RxdataB5,fs,N_sample);
[FRAC1,PRAC1]=tospec(RxconstRC1,fs,N_sample);
[FRBC1,PRBC1]=tospec(RxconstIC1,fs,N_sample);
[FRAC2,PRAC2]=tospec(RxconstRC2,fs,N_sample);
[FRBC2,PRBC2]=tospec(RxconstIC2,fs,N_sample);
Rxraise=comm.RaisedCosineReceiveFilter('Shape',...
    'Square root','RolloffFactor',RollOffFactor,'InputSamplesPerSymbol',N_sample,'Gain',1,'DecimationFactor',N_sample,'FilterSpanInSymbols',1);
RxdataA151=step(Rxraise,RxdataA15);%pass the received inphase signal through raised cosin square root filter receiver for the 15dB awgn channel case
RxdataB151=step(Rxraise,RxdataB15);%pass the received quadrature signal through raised cosin square root filter receiver for the 15dB awgn channel case
RxdataA51=step(Rxraise,RxdataA5);%pass the received inphase signal through raised cosin square root filter receiver for the 5dB awgn channel case
RxdataB51=step(Rxraise,RxdataB5);%pass the received quadrature signal through raised cosin square root filter receiver for the 5dB awgn channel case

release(Rxraise);% for implementing receiver of the complex signal use 'release'
Rxconst15=step(Rxraise,de_const15);
Rxconst5=step(Rxraise,de_const5);
step(cd6,Rxconst5);%draw the constellation
step(cd4,Rxconst15);%draw the constellation

RxconstC1=step(Rxraise,de_constC1);
RxconstC2=step(Rxraise,de_constC2);
release(Rxraise);
RxconstAC1=step(Rxraise,real(de_constC1));%for the drawing of the constellation we also pass the inphase demodulated signal for case 1 of the multipath channel
RxconstBC1=step(Rxraise,imag(de_constC1));%for the drawing of the constellation we also pass the quadrature demodulated signal for case 1 of the multipath channel
RxconstAC2=step(Rxraise,real(de_constC2));%for the drawing of the constellation we also pass the inphase demodulated signal for case 1 of the multipath channel
RxconstBC2=step(Rxraise,imag(de_constC2));%for the drawing of the constellation we also pass the quadrature demodulated signal for case 1 of the multipath channel
step(cd7,RxconstC1);
step(cd8,RxconstC2);


for i=1:length(RxdataA151)%this part receive the two 2ASK signal and map them to bipolar integer representation
    if (RxdataA151(i)<0);
        SumRA(i)=-1;
    elseif (RxdataA151(i)>0);
        SumRA(i)=1;
    else
        SumRA(i)=-1;
    end
    
    if (RxdataB151(i)<0);
        SumRB(i)=-1;
    elseif (RxdataB151(i)>0);
        SumRB(i)=1;
    else
        SumRB(i)=-1;
    end
end
for i=1:length(O_data)%this part maps the transmitted signal to bipolar representation for BER estimation
    if O_data(i)==0;
        O_dataT(2*i-1)=-1;
        O_dataT(2*i)=-1;
    elseif O_data(i)==1;
        O_dataT(2*i-1)=-1;
        O_dataT(2*i)=1;
    elseif O_data(i)==2;
        O_dataT(2*i-1)=1;
        O_dataT(2*i)=-1;
    else O_data(i)==3;
        O_dataT(2*i-1)=1;
        O_dataT(2*i)=1;
    end
end

for i=1:length(SumRB)%convert the signal from parallel to serial
    SumR(2*i-1)=SumRB(i);
    SumR(2*i)=SumRA(i);
end
%conduct BER estimatein a combined and seperated manner
[Errcount,BER,Missync]=BERestimate(SumR,O_dataT)%the 1st input parameter should be the received signal the 2nd original signal

[ErrcountA,BERA,MissyncA]=BERestimate(SumRA,a)%conduct BER estimation of only the Inphase signal 
[ErrcountB,BERB,MissyncB]=BERestimate(SumRB,b)%conduct BER estimation of only the quadrature signal 

%-------this part is for figure generation--------%
figure(1)
subplot(5,3,1)
plot(FT,PT);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of the transmitted signal after modulation');

subplot(5,3,2)
plot(F15,P15);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of signal after 15dB AWGN channel');

subplot(5,3,3)
plot(F5,P5);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of signal after 5dB AWGN channel');

subplot(5,3,4)
plot(FM1,PM2);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of signal after case1 multipath channel');

subplot(5,3,5)
plot(FM2,PM2);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of signal after case2 multipath channel');

subplot(5,3,6)
plot(FRA15,PRA15);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Inphase signal after demodulation for 15dB AWGN case');

subplot(5,3,7)
plot(FRB15,PRB15);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Quadrature signal after demodulation for 15dB AWGN case');

subplot(5,3,8)
plot(FRA5,PRA5);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Inphase signal after demodulation for 5dB AWGN case');

subplot(5,3,9)
plot(FRB5,PRB5);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Quadrature signal after demodulation for 5dB AWGN case');
subplot(5,3,10)
plot(FRAC1,PRAC1);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Inphase signal after demodulation for case 1 multipath channel');

subplot(5,3,11)
plot(FRBC1,PRBC1);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Quadrature signal after demodulation for case 1 multipath channel');

subplot(5,3,12)
plot(FRAC2,PRAC2);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Inphase signal after demodulation for case 2 multipath channel');

subplot(5,3,13)
plot(FRBC2,PRBC2);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Spectrum of parallel Quadrature signal after demodulation for case 2 multipath channel');



h1=eyediagram(RxdataA151,2);
title('Eye diagram of parallel signal Inphase for 15dB AWGN channel case');
%print(h1,'G:\CIS-research\HW1\simulation results\QPSK\Eye diagram of parallel signal Inphase for 15dB AWGN channel case.jpeg','-djpeg','-r300')

h2=eyediagram(RxdataB151,2);
title('Eye diagram of parallel signal Quadrature for 15dB AWGN channel case');
%print(h2,'G:\CIS-research\HW1\simulation results\QPSK\Eye diagram of parallel signal Quadrature for 15dB AWGN channel case.jpeg','-djpeg','-r300')

h3=eyediagram(RxdataA51,2);
title('Eye diagram of parallel signal Inphase for 5dB AWGN channel case');
%print(h3,'G:\CIS-research\HW1\simulation results\QPSK\Eye diagram of parallel signal Inphase for 5dB AWGN channel case.jpeg','-djpeg','-r300')


h4=eyediagram(RxdataB51,2);
title('Eye diagram of parallel signal Quadrature for 5dB AWGN channel case');
%print(h4,'G:\CIS-research\HW1\simulation results\QPSK\Eye diagram of parallel signal Quadrature for 5dB AWGN channel case.jpeg','-djpeg','-r300')

h5=eyediagram(RxconstAC1,2);
title('Eye diagram of parallel signal Inphase for case 1 multipath channel');
%print(h5,'G:\CIS-research\HW1\simulation results\QPSK\Eye diagram of parallel signal Inphase for case 1 multipath channel.jpeg','-djpeg','-r300')

h6=eyediagram(RxconstBC1,2);
title('Eye diagram of parallel signal Quadrature for case 1 multipath channel');
%print(h6,'G:\CIS-research\HW1\simulation results\QPSK\Eye diagram of parallel signal Quadrature for case 1 multipath channel.jpeg','-djpeg','-r300')


h7=eyediagram(RxconstAC2,2);
title('Eye diagram of parallel signal Inphase for case 2 multipath channel');
%print(h7,'G:\CIS-research\HW1\simulation results\QPSK\Eye diagram of parallel signal Inphase for case 2 multipath channel.jpeg','-djpeg','-r300')

h8=eyediagram(RxconstBC2,2);
title('Eye diagram of parallel signal Quadrature for case 2 multipath channel');
%print(h8,'G:\CIS-research\HW1\simulation results\QPSK\Eye diagram of parallel signal Quadrature for case 2 multipath channel.jpeg','-djpeg','-r300')















        
