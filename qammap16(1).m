%%
%%Author information
%Patrick(YuqiMeng) in CIS online program under professor Danijela Cabric 2019.10.17 version1.0
%%Brief description of its function
%this function mapps the decimal data to 2 Quadrature 4ASK data that is
%used to be modulated to become transmitted data, the message contained in the
%'O_data' are rearranged into 'a','b' in a way which the adjacent symbol
%in the constellation have only one different bit which follows the gray
%mapping method.This is supposed to have a minimum BER
function [a,b]=qammap16(O_data)
for i=1:length(O_data)%map the signal to I and Q 'a' is assumed to be Inphase
    if O_data(i)==0;
        a(i)=-1;
        b(i)=-1;
    elseif O_data(i)==1;
        a(i)=-3;
        b(i)=-1;
    elseif O_data(i)==2;
        a(i)=-1;
        b(i)=-3;
    elseif O_data(i)==3;
        a(i)=-3;
        b(i)=-3;
    elseif O_data(i)==4;
        a(i)=1;
        b(i)=-1;
    elseif O_data(i)==5;
        a(i)=1;
        b(i)=-3;
    elseif O_data(i)==6;
        a(i)=3;
        b(i)=-1;
    elseif O_data(i)==7;
        a(i)=3;
        b(i)=-3;
    elseif O_data(i)==8;
        a(i)=-1;
        b(i)=1;
    elseif O_data(i)==9;
        a(i)=-1;
        b(i)=3;
    elseif O_data(i)==10;
        a(i)=-3;
        b(i)=1;
    elseif O_data(i)==11;
        a(i)=-3;
        b(i)=3;
    elseif O_data(i)==12;
        a(i)=1;
        b(i)=1;
    elseif O_data(i)==13;
        a(i)=3;
        b(i)=1;
    elseif O_data(i)==14;
        a(i)=1;
        b(i)=3;
    elseif O_data(i)==15;
        a(i)=3;
        b(i)=3;
    end
end
end