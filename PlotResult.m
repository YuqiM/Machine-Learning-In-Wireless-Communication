clear all vars 
close all
clc
[filename,path]=uigetfile();%interactive tool for obtaining filename
if ischar(path)~=1
    return
end
prompt={'Select if the data is linear spacing if so type in "yes"'};
dlg_title = 'Choose data type';
num_lines = 1;
answer = inputdlg(prompt,dlg_title,num_lines);
Loopm=-0.3:0.05:-0.3
Unit=(max(Loopm)-min(Loopm))/((length(Loopm)-1));
if strcmp(answer,'yes')
    for k=Loopm
        figure(double(int8((k+abs(Loopm(1)))/Unit))+1)
        for j=1:3
        Tt=importdata(strcat(path,'drop',num2str(j),'TER3',num2str(7.5),'R2',num2str(4.5),'w0.4','R1',num2str(7.5),'um.mat'));
        %Tt=importdata(strcat(path,filename));
        T(1,:)=abs(Tt.T(1,1,:));%Make the variable positive for convenience for future processing
        lambda(1,:)=Tt.lambda;
        um=1e-6;
        if j==1
        R(int8(3*((k+abs(Loopm(1)))/Unit)+j))=7.5*um;
        elseif j==2
        R(int8(3*((k+abs(Loopm(1)))/Unit)+j))=(4.5)*um;
        else
        R(int8(3*((k+abs(Loopm(1)))/Unit)+j))=(4.5+k)*um; 
        end
        tag=0;
        flag=1;
        
        [pks,locs,w,p]=findpeaks(T,'MinPeakHeight',0.1*max(T(find(T<max(T)))),'WidthReference','halfheight');%Half to second largest
        [pks,locs,w,p]=findpeaks(T,'MinPeakHeight',0.1*max(T(find(T<max(T)))),'WidthReference','halfheight');%Half to second largest
        
         posi=lambda(locs);
       
         for i=1:(length(posi)-1)
            FSR(i)=posi(i+1)-posi(i);
         end
         FWHM=w*(0.4*um/3e4);
        %if j==2
        %figure(double(int8((k+abs(Loopm(1)))/0.1))+1)
        plot(lambda,T,'-^');
        hold on
        try
%         plot(lambda(locs-floor(w/2)),10*log10(pks/2),'o');
%         plot(lambda(locs+floor(w/2)-2),10*log10(pks/2),'o');
        legend('Drop R1','Drop R2','Drop R3');
        xlabel('Wavelength(m)');
        ylabel('Transmission');
        title(strcat('Transmission spectrum at drop2 for',{' '},num2str(k),'um2nd ring'));
        grid on
        %hold off
        catch ME
        end
   
        %end
        FWHM
        FSR
        Tag=find(abs(posi-1.55*um)==min(abs(posi-1.55*um)));
        L(int8(2*((k+abs(Loopm(1)))/Unit)+j))=2*pi*R(int8(2*((k+abs(Loopm(1)))/Unit)+j));
        if (Tag==length(posi))
            Tag=Tag-1;
        end
        
        FWHMS(int8(3*((k+abs(Loopm(1)))/Unit))+j)=FWHM(Tag)
        Lambdares(int8(3*((k+abs(Loopm(1)))/Unit))+j)=posi(Tag)
        FSRS(int8(3*((k+abs(Loopm(1)))/Unit)+j))=FSR(Tag) 
        Ng(int8(3*((k+abs(Loopm(1)))/Unit)+j))=posi(Tag)^2/(L(int8(3*((k+abs(Loopm(1)))/Unit)+j))*FSR(Tag));
        end
    end
    
else
    for imp=2.5:2.5:7*2.5
        [pks,locs,w,p]=findpeaks(T,'MinPeakHeight',0.6,'WidthReference','halfheight');
        posi=lambda(locs);
        for i=1:(length(posi)-1)
             FSR(i)=posi(i)-posi(i+1);
        end
        FWHM=w*(0.4*um/1e4);
        plot(lambda,T,'-^');
        hold on
        plot(lambda(locs-floor(w/2)),pks/2,'o');
        plot(lambda(locs+floor(w/2)),pks/2,'o');
        xlabel('Wavelength(m)');
        grid on
        hold off
        FWHM
        FSR
        Tag=find(abs(posi-1.55*um)==min(abs(posi-1.55*um)));
        FWHMS(imp/2.5)=FWHM(Tag)
        Lambdares(imp/2.5)=posi(Tag)
        FSRS(imp/2.5)=FSR(Tag-1) 
        Ng(imp/2.5)=posi(Tag)^2/(L*FSR(Tag));
    end
end
% R=[-0.5:0.1:0.5];
% Q=Lambdares./FWHMS;
% figure(length(R)+1)
% subplot(2,2,1);
% plot(R,FSRS(2:2:length(R)*2)/(1e-9),'-o','MarkerSize',12);
% hold on
% plot(R,FSRS(1:2:(length(R)*2-1))/(1e-9),'-o','MarkerSize',12);
% hold off
% xlabel('Radius(um)');
% ylabel('FSR(nm)');
% grid on
% title('FSR change in relation to Ring Radius around 1550nm');
% legend('FSR2','FSR1');
% subplot(2,2,2);
% plot(R,Q(2:2:length(R)*2),'-o','MarkerSize',12);
% hold on
% plot(R,Q(1:2:(length(R)*2-1)),'-o','MarkerSize',12);
% hold off
% xlabel('Radius(um)');
% ylabel('Q');
% grid on
% title('Q factor change in relation to Ring Radius around 1550nm');
% legend('drop2','drop1');
% subplot(2,2,3);
% plot(R,Ng(2:2:length(R)*2),'-o','MarkerSize',12);
% hold on
% plot(R,Ng(1:2:(length(R)*2-1)),'-o','MarkerSize',12);
% hold off
% legend('drop2','drop1');
% xlabel('Radius(um)');
% ylabel('Ng');
% grid on
% title('Group index(Ng) change in relation to Ring Radius around 1550nm');
% subplot(2,2,4);
% plot(R,Lambdares(2:2:length(R)*2)/(1e-9),'-ro','MarkerSize',12);
% hold on
% plot(R,Lambdares(1:2:(length(R)*2-1))/(1e-9),'-o','MarkerSize',12);
% hold off
% xlabel('Radius(um)');
% ylabel('Lambdares(nm)');
% legend('drop2','drop1');
% grid on
% title('Lambdares change in relation to Ring Radius around 1550nm');

