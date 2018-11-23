clc; clear; close all;
clf
load('stereoParams.mat')
cam1 = webcam(2);
cam2 = webcam(3);
vid1 = cam1;
vid2 = cam2;
Thresh_r = 40;
Thresh_g = 40;
Thresh_b = 40;
Num_pixels = 480*640;

red_n1 = zeros(480,640);
green_n1 = zeros(480,640);
blue_n1 = zeros(480,640);

red_n2 = zeros(480,640); 
green_n2 = zeros(480,640);
blue_n2 = zeros(480,640);

Slop = 15;

Uno = arduino();
configurePin(Uno,'D2','DigitalOutput'); %Waist Speed
configurePin(Uno,'D24','DigitalOutput'); %Waist Dir1
configurePin(Uno,'D26','DigitalOutput'); %Waist Dir2
configurePin(Uno,'D4','DigitalOutput'); %ElbowSpeed
configurePin(Uno,'D33','DigitalOutput'); %ElbowDir1
configurePin(Uno,'D35','DigitalOutput'); %ElbowDir2
configurePin(Uno,'D5','DigitalOutput'); %WristSpeed
configurePin(Uno,'D37','DigitalOutput'); %WristDir1
configurePin(Uno,'D39','DigitalOutput'); %WristDir2

isCentered = 0;

%Target Tracking section
while(isCentered == 0)
    pic1 = snapshot(vid1);
    red1 = pic1(:,:,1);
    green1 = pic1(:,:,2);
    blue1 = pic1(:,:,3);
    pic2 = snapshot(vid2);
    red2 = pic2(:,:,1);
    green2 = pic2(:,:,2);
    blue2 = pic2(:,:,3);
    
    for i = 1:Num_pixels
        if red1(i) >= Thresh_r
            red_n1(i) = 1;%red(i);
        else
            red_n1(i) = 0;
        end
        
        if blue1(i)>= Thresh_b
            blue_n1(i) = 1;%blue(i);
        else
            blue_n1(i) = 0;
        end
        
        if green1(i)>=Thresh_g
            green_n1(i) = 1;%green(i);
        else
            green_n1(i) = 0;
        end
    end
    
    for i = 1:Num_pixels
        if red2(i) >= Thresh_r
            red_n2(i) = 1;%red(i);
        else
            red_n2(i) = 0;
        end
        
        if blue2(i)>= Thresh_b
            blue_n2(i) = 1;%blue(i);
        else
            blue_n2(i) = 0;
        end
        
        if green2(i)>=Thresh_g
            green_n2(i) = 1;%green(i);
        else
            green_n2(i) = 0;
        end
    end

    [r31,c31] = find((red1<Thresh_r)&(green1<Thresh_g)&(blue1<Thresh_b));
    massblack1 = length(r31);
    avgRblack1 = round(mean(r31));
    avgCblack1 = round(mean(c31));
    
    [r32,c32] = find((red2<Thresh_r)&(green2<Thresh_g)&(blue2<Thresh_b));
    massblack2 = length(r32);
    avgRblack2 = round(mean(r32));
    avgCblack2 = round(mean(c32));
% [avgRblack1,avgCblack1] = imXYfind(cam1)
% [avgRblack2,avgCblack2] = imXYfind(cam2)
    
     D=triangulate([avgCblack1,480-avgRblack1],[avgCblack2,480-avgRblack2],stereoParams);
     vec = D;
    
    x = vec(1)-50;
    y = vec(2);
    z = vec(3);
    
    disp('Distance from robot (mm):'); disp(z); disp('Vertical Distance (mm):'); disp(y);
 
    
    if x <= -1* Slop
        writeDigitalPin(Uno,'D24',0);
        writeDigitalPin(Uno,'D26',1);
        writePWMVoltage(Uno,'D2',2.1);
    elseif x >= Slop
        writeDigitalPin(Uno,'D24',1);
        writeDigitalPin(Uno,'D26',0);
        writePWMVoltage(Uno,'D2',2.1);
    else
        writePWMVoltage(Uno,'D2',0);
        isCentered = 1;
    end

  

    
    
    pic_n1(:,:,1) = red_n1;
    pic_n1(:,:,2) = green_n1;
    pic_n1(:,:,3) = blue_n1;
    pic_n2(:,:,1) = red_n2;
    pic_n2(:,:,2) = green_n2;
    pic_n2(:,:,3) = blue_n2;
    
figure(1)
    subplot(3,2,1)
    hold on
    imagesc(flipud(pic_n1)); title('Blob Detection Camera 1')
    plot(avgCblack1,480-avgRblack1, 'r+', 'MarkerSize', 30, 'LineWidth', 2); axis([0 640 0 480]);
    plot(320,240, 'g+', 'MarkerSize', 20, 'LineWidth', 1);
    hold off
    subplot(3,2,2)
    imagesc(pic1); axis([0 640 0 480]);  title('Camera 1 Display')
 
    
    subplot(3,2,3)
    hold on
    imagesc(flipud(pic_n2));  title('Blob Detection Camera 2')
    plot(avgCblack2,480-avgRblack2, 'r+', 'MarkerSize', 30, 'LineWidth', 2); axis([0 640 0 480]);
    plot(320,240, 'g+', 'MarkerSize', 20, 'LineWidth', 1);
    hold off
    subplot(3,2,4)
    imagesc(pic2); axis([0 640 0 480]);   title('Camera 2 Display');
    
    Theta = FindTheta(D(3),D(2),9.14);

    V=9.14;
a = -9.81;    
    tfinal =(D(3)/1000)/(V*cosd(Theta));
t = 0:.01:tfinal;
xp = V*cosd(Theta)*t;
yp = V*sind(Theta)*t+.5*a*t.^2;

h1 = subplot(3,2,5);
%cla(h1)
hold on
plot(xp,yp); xlabel('X'); ylabel('Y')
plot(D(3)/1000,D(2)/1000,'r*')
hold off

h1=subplot(3,1,3);
hold on
plot(xp,yp); xlabel('X'); ylabel('Y')
plot(D(3)/1000,D(2)/1000,'r*')
title('Trajectory Calculations')
hold off
        pause(.05)
        cla(h1)
        

end
%Trajectory Compensation section
isTraj = 0
while(isTraj==0)
    home = 2.55;
    %3.3260 = 15 deg
    %.03519 volts/degree
    WristVal = readVoltage(Uno,'A3');
    
%         writeDigitalPin(Uno,'D37',1); %Up
%         writeDigitalPin(Uno,'D39',0);
%         writePWMVoltage(Uno,'D5',2);
        
        VoltsNeeded = (Theta-10)*.036+home;
        
        if(WristVal <= VoltsNeeded -0.05)
            writeDigitalPin(Uno,'D37',1);
            writeDigitalPin(Uno,'D39',0);
            writePWMVoltage(Uno,'D5',3);
        elseif(WristVal >= VoltsNeeded + 0.05)
            writeDigitalPin(Uno,'D37',0);
            writeDigitalPin(Uno,'D39',1);
            writePWMVoltage(Uno,'D5',3);
        else
            writePWMVoltage(Uno,'D5',0);
            isTraj = 1
        end
            
end
%Firing Section
%if (isTraj==1)
pause(1)
writeDigitalPin(Uno,'D13',1);
writeDigitalPin(Uno,'D12',1);
pause(.15)
writeDigitalPin(Uno,'D12',0);
    
    
        
    
    