%%
%%Author information
%Patrick(YuqiMeng) in CIS online program under professor Danijela Cabric 2019.10.17 version1.0
%%Brief description of its function
%This code perform the simulation of the transmission of the BPSK signal
%first the random sequence of signal is generated then using the self
%defined function the signal is mapped to the constellation with only 1 bit
%difference between each adjacent point that approach is supposed to result
%in minimum BER. The pulse shaping filter is designed using
%square root raised cosin transmitter and corresponding receiver. The
%spectrum of each stage of the signal transmission is shown in figure
%ploted into one figure in 'subplot' expression. The drawing of constellation
%requires the signal have an imaginary part. So for convenience we send an
%addition signal using the complex representation at each tranmitter and
%receiver for mapping of constellation.And constellation of signal
%after receiver for all 4 types of channels are shown. And additionally
%constellation of the signal at some points of the system before receiver
%is also shown as it is in the modulated form its spread in the graph can be considerable
%BER: BER is calculated using the self defined function 'BERestimate' after
%the signal have gone through many stages and passes many filters.
%Missynchronization can occur and the received signal can suffer an
%additional delay. So this function comes useful in the BER
%evalution. Further information such as input and output is described
%inside the function.
%The ouput BER shown is the BER after 15dB awgn channel, for more BER
%estimate specific input of corresponding functions can be adjusted to
%calculate theirs'.
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
BW=10e3;
bitrate=BW/2;
N_sample=128;%sample rate for modulation
RollOffFactor=1;
M=2;
k=log2(M);
fs=bitrate/k;
fc=50e3;%moduate the signal onto 8MHz
O_data=randi([0 M-1],10,1);
mod = comm.BPSKModulator;
cd = comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the transmitted signal');
cd1=comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the signal after 15dB AWGN channel');
cd2=comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the signal after 5dB AWGN channel');
cd3=comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the received signal of 15dB AWGN channel');
cd4=comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the received signal of 5dB AWGN channel');
cd5=comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the received signal of Multipath 1 channel');
cd6=comm.ConstellationDiagram(...
    'ReferenceConstellation',constellation(mod), ...
    'XLimits',[-5 5],'YLimits',[-5 5],'Title','constellation of the received signal of Multipath 2 channel');
O_P=real(step(mod,O_data));
Txraise=comm.RaisedCosineTransmitFilter('Shape','Square root','RolloffFactor',...
    RollOffFactor,'OutputSamplesPerSymbol',N_sample,'Gain',1,'FilterSpanInSymbols',1);%generate the raised cosin filter object

Txdata=step(Txraise,O_P);
Txdata=Txdata*1/(max(Txdata));

Txdata1=ammod(Txdata,fc,fs*N_sample);
%sync=comm.CarrierSynchronizer('Modulation','PAM','SamplesPerSymbol',N_sample);
%[Txdata1,H]=step(sync,Txdata1);
[F1,P1]=tospec(Txdata1,fs,N_sample);
figure(1)
subplot(3,2,1)
plot(F1,P1);
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('Sectrum of the transmitted signal');
step(cd,Txdata1);



chanawgn15=comm.AWGNChannel('NoiseMethod','Signal to noise ratio (SNR)','SNR',15);%create the AWGN channel object with specified SNR
chanawgn5=comm.AWGNChannel('NoiseMethod','Signal to noise ratio (SNR)','SNR',5);
Multichan1=comm.RayleighChannel('SampleRate',N_sample,'PathDelays',[0 30e-9],'AveragePathGains',[0 -3]);%set the relative path delays to [0 30ns] relative power to [0 -3dB]
Multichan2=comm.RayleighChannel('SampleRate',N_sample,'PathDelays',[0 1e-9],'AveragePathGains',[0 -3]);%set the relative path delays to [0 1ns] relative power to [0 -3dB]
Txdata2=step(chanawgn15,Txdata1);%passing the signal through awgn channel
Txdata3=step(chanawgn5,Txdata1);%passing the signal through awgn channel
Txdata4=step(Multichan1,Txdata1);%passing the signal through multipath channel
Txdata5=step(Multichan2,Txdata1);%passing the signal through multipath channel

passf=8e6;%CenterFrequencyOfThePassbandFilter
Fp1=(passf-BW/2)/(N_sample*fs/2);
Fp2=(passf+BW/2)/(N_sample*fs/2);
Fst1=Fp1-(BW/4)/(N_sample*fs/2);
Fst2=Fp2+(BW/4)/(N_sample*fs/2);
%D=fdesign.bandpass('Fst1,Fp1,Fp2,Fst2,Ast1,Ap,Ast2',Fst1,Fp1,Fp2,Fst2,60,1,60);%design a bandpass filter to filter out the noise
%passbandf=design(D);
%Txdata2=filter(passbandf,Txdata2);
%hIQComp = comm.IQImbalanceCompensator;
%Txdata3=step(hIQComp,Txdata3);

step(cd1,Txdata2);
step(cd2,Txdata3);
subplot(3,2,2)
[F2,P2]=tospec(Txdata2,fs,N_sample);
plot(F2,P2);
xlabel('Frequency(Hz)');
ylabel('Amplitude')
title('spectrum of the signal at the 15dB AWGN channel output');
subplot(3,2,3)
[F3,P3]=tospec(Txdata4,fs,N_sample);
plot(F3,P3);
xlabel('Frequency(Hz)');
ylabel('Amplitude')
title('spectrum of the signal at the case 1 multipath channel output');

BER1=[];

hIQComp = comm.IQImbalanceCompensator;
Txdata3=step(hIQComp,Txdata3);
Rxdata=amdemod(Txdata2,fc,N_sample*fs);%demodulate the signal
Rxdata5dB=amdemod(Txdata3,fc,N_sample*fs);%demodulate the signal
RxdataMPR1=amdemod(real(Txdata4),fc,N_sample*fs);%imag part is just a representation of the relative carrier phase offset
RxdataMPI1=amdemod(imag(Txdata4),fc,N_sample*fs,pi/2);
RxdataMPC1=real(RxdataMPR1)+j*RxdataMPI1;
RxdataMPR2=amdemod(real(Txdata5),fc,N_sample*fs);%imag part is just a representation of the relative carrier phase offset
RxdataMPI2=amdemod(imag(Txdata5),fc,N_sample*fs,pi/2);
RxdataMPC2=real(RxdataMPR2)+j*RxdataMPI2;

%Bpassfilter==filterbuilder
%Rxdata=pamdemod(Txdata2,4);
Rxraise=comm.RaisedCosineReceiveFilter('Shape',...
    'Square root','RolloffFactor',RollOffFactor,'InputSamplesPerSymbol',N_sample,'Gain',1,'DecimationFactor',N_sample,'FilterSpanInSymbols',1);
Rxdata1=step(Rxraise,Rxdata);
Rxdata5dB1=step(Rxraise,Rxdata5dB);
release(Rxraise);
RxdataMPR1=step(Rxraise,RxdataMPC1);
step(cd5,RxdataMPR1);
RxdataMPR2=step(Rxraise,RxdataMPC2);
step(cd6,RxdataMPR2);
step(cd3,Rxdata1);
step(cd4,Rxdata5dB);
subplot(3,2,4)
[F2,P2]=tospec(Rxdata1,fs,1);
plot(F2,P2);
xlabel('Frequency(Hz)');
ylabel('Amplitude')
title('spectrum of the signal after raisedcosin receiver');
mod1=comm.BPSKDemodulator;
Rxdata2=step(mod1,Rxdata1);
[F2,P2]=tospec(Rxdata,fs,N_sample);
subplot(3,2,5)
plot(F2,P2)
xlabel('Frequency(Hz)');
ylabel('Amplitude')
title('spectrum of the signal after AM demodulation');
subplot(3,2,6)
[F2,P2]=tospec(Txdata,fs,N_sample);
plot(F2,P2)
xlabel('Frequency(Hz)');
ylabel('Amplitude')
title('spectrum of the baseband signal after raised cosin transmitter');

[Errcount,BER,Missync]=BERestimate(Rxdata2,O_data)%the 1st parameter should be the received signal the 2nd original signal

figure(2)
plot(0:(length(Txdata2)-1),Txdata2)
hold on
plot(0:(length(Rxdata)-1),Rxdata);
hold off
legend('waveform before demod','waveform after demod');

figure(3)
[F1,P1]=tospec(O_P,fs,1);
plot(F1,P1)
xlabel('Frequency(Hz)');
ylabel('Amplitude');
title('SpectrumOfDigitalBassbandSignal')

figure(4)
subplot(3,1,1)
[F2,P2]=tospec(Txdata3,fs,N_sample);
plot(F2,P2);
xlabel('Frequency(Hz)');
ylabel('Amplitude')
title('spectrum of the signal at the 5dB AWGN channel output');
subplot(3,1,2)
[F2,P2]=tospec(Rxdata5dB1,fs,1);
plot(F2,P2);
xlabel('Frequency(Hz)');
ylabel('Amplitude')
title('spectrum of the signal after raisedcosin receiver for 5dB awgn channel');
subplot(3,1,3)
[F2,P2]=tospec(Rxdata5dB,fs,N_sample);

plot(F2,P2)
xlabel('Frequency(Hz)');
ylabel('Amplitude')
title('spectrum of the signal after AM demodulation for 5dB awgn channel');


h1=eyediagram(Rxdata1,2);
title('Eye diagram of signal at the receiver for 15dB AWGN channel');
%print(h1,'G:\CIS-research\HW1\simulation results\BPSK\Eye diagram for signal at the receiver for 15dB AWGN channel.jpeg','-djpeg','-r300')

h2=eyediagram(Rxdata5dB1,2);
title('Eye diagram of signal at the receiver for 5dB AWGN channel');
%print(h2,'G:\CIS-research\HW1\simulation results\BPSK\Eye diagram for signal at the receiver for 5dB AWGN channel.jpeg','-djpeg','-r300')

h3=eyediagram(real(RxdataMPR1)+imag(RxdataMPR1),2);
title('Eye diagram of signal at the receiver for case 1 multipath channel');
%print(h3,'G:\CIS-research\HW1\simulation results\BPSK\Eye diagram of signal at the receiver for case 1 multipath channel.jpeg','-djpeg','-r300')

h4=eyediagram(real(RxdataMPR2)+imag(RxdataMPR2),2);
title('Eye diagram of signal at the receiver for case 2 multipath channel');
%print(h4,'G:\CIS-research\HW1\simulation results\BPSK\Eye diagram of signal at the receiver for case 2 multipath channel.jpeg','-djpeg','-r300')





































% fs = 1e6;           % Symbol rate (Hz)
% sps = 4;              % Samples per symbol
% M = 2;               % Modulation order
% k = log2(M);          % Bits per symbol
% mod = comm.RectangularQAMModulator(M);
% chan = comm.AWGNChannel('EbNo',100,'BitsPerSymbol',k,'SamplesPerSymbol',sps);
% cd = comm.ConstellationDiagram(...
%     'ReferenceConstellation',constellation(mod), ...
%     'XLimits',[-5 5],'YLimits',[-5 5]);
% fOffset = comm.PhaseFrequencyOffset(...
%     'FrequencyOffset',400,...
%     'PhaseOffset',20,...
%     'SampleRate',fs);
% data = randi([0 M-1],1000,1);
% txSig = step(mod,data);
% rxSig = step(chan,step(fOffset,txSig));
% csync = comm.CarrierSynchronizer('DampingFactor',0.7, ...
%     'NormalizedLoopBandwidth',0.005, ...
%     'SamplesPerSymbol',sps, ...
%     'Modulation','QAM');
% rxData = step(csync,rxSig);
% step(cd,rxData(501:1000))
% eyediagram(rxSig,2)