%%
%%Author information
%Patrick(YuqiMeng) in CIS online program under professor Danijela Cabric 2019.10.17 version1.0
%%Brief description of its function
%this function mapps the originally sent in the decimal system into binary
%system for BER estimation



function BERsent=qam16tobit(O_data)
for i=1:length(O_data)
    if O_data(i)==0;
        BERsent(4*i-3)=0;
        BERsent(4*i-2)=0;
        BERsent(4*i-1)=0;
        BERsent(4*i)=0;
        
    elseif O_data(i)==1;
        BERsent(4*i-3)=0;
        BERsent(4*i-2)=0;
        BERsent(4*i-1)=0;
        BERsent(4*i)=1;
    elseif O_data(i)==2;
        BERsent(4*i-3)=0;
        BERsent(4*i-2)=0;
        BERsent(4*i-1)=1;
        BERsent(4*i)=0;
    elseif O_data(i)==3;
        BERsent(4*i-3)=0;
        BERsent(4*i-2)=0;
        BERsent(4*i-1)=1;
        BERsent(4*i)=1;
    elseif O_data(i)==4;
        BERsent(4*i-3)=0;
        BERsent(4*i-2)=1;
        BERsent(4*i-1)=0;
        BERsent(4*i)=0;
    elseif O_data(i)==5;
        BERsent(4*i-3)=0;
        BERsent(4*i-2)=1;
        BERsent(4*i-1)=0;
        BERsent(4*i)=1;
    elseif O_data(i)==6;
        BERsent(4*i-3)=0;
        BERsent(4*i-2)=1;
        BERsent(4*i-1)=1;
        BERsent(4*i)=0;
    elseif O_data(i)==7;
        BERsent(4*i-3)=0;
        BERsent(4*i-2)=1;
        BERsent(4*i-1)=1;
        BERsent(4*i)=1;
    elseif O_data(i)==8;
        BERsent(4*i-3)=1;
        BERsent(4*i-2)=0;
        BERsent(4*i-1)=0;
        BERsent(4*i)=0;
    elseif O_data(i)==9;
        BERsent(4*i-3)=1;
        BERsent(4*i-2)=0;
        BERsent(4*i-1)=0;
        BERsent(4*i)=1;
    elseif O_data(i)==10;
       BERsent(4*i-3)=1;
        BERsent(4*i-2)=0;
        BERsent(4*i-1)=1;
        BERsent(4*i)=0;
    elseif O_data(i)==11;
        BERsent(4*i-3)=1;
        BERsent(4*i-2)=0;
        BERsent(4*i-1)=1;
        BERsent(4*i)=1;
    elseif O_data(i)==12;
        BERsent(4*i-3)=1;
        BERsent(4*i-2)=1;
        BERsent(4*i-1)=0;
        BERsent(4*i)=0;
    elseif O_data(i)==13;
        BERsent(4*i-3)=1;
        BERsent(4*i-2)=1;
        BERsent(4*i-1)=0;
        BERsent(4*i)=1;
    elseif O_data(i)==14;
        BERsent(4*i-3)=1;
        BERsent(4*i-2)=1;
        BERsent(4*i-1)=1;
        BERsent(4*i)=0;
    elseif O_data(i)==15;
        BERsent(4*i-3)=1;
        BERsent(4*i-2)=1;
        BERsent(4*i-1)=1;
        BERsent(4*i)=1;
    end
end
end