%%
%%Author information
%Patrick(YuqiMeng) in CIS online program under professor Danijela Cabric 2019.10.17 version1.0
%%Brief description of its function
%this function recombines two parts of the 4ASK signal used to geenrate the
%16QAM signal, regarding to the received signal, the bits are rearranged in
%a way that recovers the signal that is originally sent which is mapped in
%constellation to assume minimum BER.


function BERreceive=qam16recombine(a,b)
  for i=1:length(a)
      if (a(i)==1)&&(b(i)==1)
          
          BERreceive(4*i-3)=1;
          BERreceive(4*i-2)=1;
          BERreceive(4*i-1)=0;
          BERreceive(4*i)=0;
      elseif (a(i)==3)&&(b(i)==1)
          
          BERreceive(4*i-3)=1;
          BERreceive(4*i-2)=1;
          BERreceive(4*i-1)=0;
          BERreceive(4*i)=1;
      elseif (a(i)==1)&&(b(i)==3)
          
          BERreceive(4*i-3)=1;
          BERreceive(4*i-2)=1;
          BERreceive(4*i-1)=1;
          BERreceive(4*i)=0;
      elseif (a(i)==3)&&(b(i)==3)
          
          BERreceive(4*i-3)=1;
          BERreceive(4*i-2)=1;
          BERreceive(4*i-1)=1;
          BERreceive(4*i)=1;
      elseif (a(i)==-1)&&(b(i)==1)
          
          BERreceive(4*i-3)=1;
          BERreceive(4*i-2)=0;
          BERreceive(4*i-1)=0;
          BERreceive(4*i)=0;
      elseif (a(i)==-3)&&(b(i)==1)
          
          BERreceive(4*i-3)=1;
          BERreceive(4*i-2)=0;
          BERreceive(4*i-1)=1;
          BERreceive(4*i)=0;
      elseif (a(i)==-1)&&(b(i)==3)
          
          BERreceive(4*i-3)=1;
          BERreceive(4*i-2)=0;
          BERreceive(4*i-1)=0;
          BERreceive(4*i)=1;
      elseif (a(i)==-3)&&(b(i)==3)
          
          BERreceive(4*i-3)=1;
          BERreceive(4*i-2)=0;
          BERreceive(4*i-1)=1;
          BERreceive(4*i)=1;
      elseif (a(i)==-1)&&(b(i)==-1)
          
          BERreceive(4*i-3)=0;
          BERreceive(4*i-2)=0;
          BERreceive(4*i-1)=0;
          BERreceive(4*i)=0;
      elseif (a(i)==-3)&&(b(i)==-1)
          
          BERreceive(4*i-3)=0;
          BERreceive(4*i-2)=0;
          BERreceive(4*i-1)=0;
          BERreceive(4*i)=1;
      elseif (a(i)==-1)&&(b(i)==-3)
          
          BERreceive(4*i-3)=0;
          BERreceive(4*i-2)=0;
          BERreceive(4*i-1)=1;
          BERreceive(4*i)=0;
      elseif (a(i)==-3)&&(b(i)==-3)
          
          BERreceive(4*i-3)=0;
          BERreceive(4*i-2)=0;
          BERreceive(4*i-1)=1;
          BERreceive(4*i)=1;
      elseif (a(i)==1)&&(b(i)==-1)
          
          BERreceive(4*i-3)=0;
          BERreceive(4*i-2)=1;
          BERreceive(4*i-1)=0;
          BERreceive(4*i)=0;
      elseif (a(i)==3)&&(b(i)==-1)
          
          BERreceive(4*i-3)=0;
          BERreceive(4*i-2)=1;
          BERreceive(4*i-1)=1;
          BERreceive(4*i)=0;
      elseif (a(i)==1)&&(b(i)==-3)
          
          BERreceive(4*i-3)=0;
          BERreceive(4*i-2)=1;
          BERreceive(4*i-1)=0;
          BERreceive(4*i)=1;
      elseif (a(i)==3)&&(b(i)==-3)
          
          BERreceive(4*i-3)=0;
          BERreceive(4*i-2)=1;
          BERreceive(4*i-1)=1;
          BERreceive(4*i)=1;
      end
  end
          
          
          