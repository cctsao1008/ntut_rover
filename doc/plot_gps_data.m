data = xlsread('gps.xlsx');

%figure(1)
format long
hold on
%plot(data(:,1),data(:,4),'r');
subplot(2,2,[1 2]),plot(data(:,1),data(:,4),'r',data(:,2),data(:,5), 'g',data(:,3),data(:,6), 'b');
title('GPS Data Plot');
xlabel('Latitude');
ylabel('Longitude');

%figure(2)
%plot(data(:,2),data(:,5), 'g');
%subplot(2,2,[1 2]),plot(data(:,2),data(:,5), 'g');
%title('GPS Data Plot');
%xlabel('Latitude');
%ylabel('Longitude');

%figure(3)
%plot(data(:,3),data(:,6), 'b');
%subplot(2,2,[1 2]),plot(data(:,3),data(:,6), 'b');
%title('GPS Data Plot');
%xlabel('Latitude');
%ylabel('Longitude');

%figure(4)
%format long
%hold on
%t = linspace(1,1,1068);
t = 1: 1 : 2340; 

%plot(t, data(:,1), 'r', t, data(:,2), 'g', t, data(:,3), 'b');
subplot(2,2,3),plot(t, data(:,1), 'r', t, data(:,2), 'g', t, data(:,3), 'b');
title('GPS Data Plot');
ylabel('Latitude');

%figure(5)
%plot(t, data(:,4), 'r', t, data(:,5), 'g', t, data(:,6), 'b');
subplot(2,2,4),plot(t, data(:,4), 'r', t, data(:,5), 'g', t, data(:,6), 'b');
title('GPS Data Plot');
ylabel('Longitude');
%legned('Latitude','Latitude-avg','Latitude-kf')

%plot(t, data(:,2))
%plot(t, data(:,3))
