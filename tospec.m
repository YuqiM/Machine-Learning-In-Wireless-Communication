%%
%%Author information
%Patrick(YuqiMeng) in CIS online program under professor Danijela Cabric 2019.10.17 version1.0
%This function makes for a convenient case where the specific data
%required to generate the spectrum of a signal is obtained by specifying
%the 'fs'(symbol rate)and 'N_sample'(samplerate) the frequency range is in
%the units(Hz) for better visualization
function [F1,P1]=tospec(Txdata1,fs,N_sample)%generate the frequency points in Hz and corresponding amplitude of the signal's FFT
%N=ceil(log2(length(Txdata1)));
%if mod(N,2)==1
   % N=N+1;
%end
L1=length(fft(Txdata1));
P2=abs(fft(Txdata1)/L1);
P1 = P2(1:round(L1/2)+1);
P1(2:end-1) = 2*P1(2:end-1);

%P1=2*abs(fft(Txdata1)/L1);%amplitude of the fourier transform
%P1=P1(1:round(L1/2));
F1=fs*N_sample*1*(0:(round((L1/2))))/L1;
end