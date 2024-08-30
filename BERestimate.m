%%
%%Author information
%Patrick(YuqiMeng) in CIS online program under professor Danijela Cabric 2019.10.17 version1.0
%%Brief description of its function

%output and input have the same length however due to some error induced in
%the receiver raised cosin filter and other filtering process
%bit sequences are not perfectly synchronized this function calculate the
%BER considering the delay in the received signal
%input: the 'output' should be the received signal and 'origina' should be the
%originally sent sequence
function [number,BER,delay]=BERestimate(output,original)%the output  'number' is the number of total counted error bits and BER 
%is the signal's bit error rate considering the error introduced in mis
%synchronization in which case the 'number' is always greater than 'delay' delay is the calculated recieved signal bit sequence's
%delay with regard to the original signal
BER=zeros(1,length(output));
 for i=1:length(output)
     for j=i:length(output)
         if output(j)~=original(j+1-i)
             BER(i)=BER(i)+1;
         end
     end
     BER(i)=BER(i)+i-1;
 end
 min=BER(1);
 for i=1:length(BER)
     if BER(i)<=min
         min=BER(i);%calculate the BER when synchronization occurs
         number=min;
         delay=i-1;%calculate the resulting delay in symbols of receiver sequences and original sequences
     end
 end
BER=(min)/length(original);
 