%%
%%Author information
%Patrick(YuqiMeng) in CIS online program under professor Danijela Cabric 2019.10.17 version1.0
%This function makes for a convenient case where the specific data
%required to generate the spectrum of a signal is obtained by specifying
%the 'fs'(symbol rate)and 'N_sample'(samplerate) the frequency range is in
%the units(Hz) for better visualization
function [F1,P1]=tospec(Txdata1,fs,N_sample)%generate the frequency points in Hz and corresponding amplitude of the signal's FFT
L1=length(fft(Txdata1));
P1=2*abs(fft(Txdata1)/L1);%amplitude of the fourier transform
P1=P1(1:L1/2);
F1=fs*N_sample*1*(0:((L1/2)-1))/L1;
end