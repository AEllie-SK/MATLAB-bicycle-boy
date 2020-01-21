clear, clc, close all;

%% home
mx = [12 12 7 7 12 15 15 17 17 19 19 15 7 12 19];
my = [5 15 15 5 5 8 15 16.5 9.5 11 20 20 15 15 20];
gx = [27 21 20 24 28 27 27 26 26 26 25 25 23 23 22 22 22 21 21];
gy = [20 20 20 23 20 20 16 16 11 16 16 19 19 16 16 11 16 16 20];

%% road
roadx = [0 40];
roady = [6 6];

%% school
roof_X = [85 79 60 59 80 86 86 66 65 83];
roof_Y = [35 20 20 15 15 30 35 35 30 30];
school_X = [60 60 80 80 80 85 85];
school_Y = [15 5 5 15 5 20 28];
doors_X = [66 66 69 69 72 72 75 75 78 78 79.5];
doors_Y = [30 21 21 27 27 21 21 27 27 21 21];
w0X = [62 62 65 65 62];
w0Y = [12 9 9 12 12];
w1X = [68 68 71 71 68];
w1Y = [12 9 9 12 12];
w2X = [74 74 77 77 74];
w2Y = [12 9 9 12 12];

%% Bicycle

n = 100;%number of points on circle 
theta = linspace(0,2*pi,n);
theta = fliplr(theta);%returns theta with items flipped on vertical axis

pedal_radius = 1.1;
back_pedal_radius = 0.5;
outer_pedal_radius = 1.5;

wheel_radius = 3.3;

pedal_X = 32 + pedal_radius * cos(theta); % x axis for pedal
pedal_Y = 7+pedal_radius * sin(theta); % y axis for pedal

outer_pedal_X = 32+outer_pedal_radius * cos(theta); % x axis for pedal
outer_pedal_Y = 7+outer_pedal_radius * sin(theta); % y axis for pedal

back_pedal_X = 25+back_pedal_radius * cos(theta); % x axis for pedal
back_pedal_Y = 7+back_pedal_radius * sin(theta); % y axis for pedal

front_wheel_X = 39+wheel_radius * cos(theta); % x axis for front wheel
front_wheel_Y = 7+wheel_radius * sin(theta); % y axis for front wheel

back_wheel_X = 25+wheel_radius * cos(theta); % x axis for back wheel
back_wheel_Y = 7+wheel_radius * sin(theta); % y axis for back wheel

upper_bike_chain_X = [25, 32];
upper_bike_chain_Y = [7.5, 8.1];

lower_bike_chain_X = [25, 32];
lower_bike_chain_Y = [6.5, 5.9];

bike_chasis_X = [32,25,27,37,32,27];
bike_chasis_Y = [7,7,11,11,7,11];

steering_rod_X = [39,36.5,35];
steering_rod_Y = [7,12,12];

for k = 1:n
    %home plot
     head_x=32;
    head_y=14.5;
    r =1;
    th = 0:pi/100:2*pi;
    xunit = r * cos(th) + head_x;
    yunit = r * sin(th) + head_y;
    plot(xunit, yunit,'LineWidth', 2,'Color', 'black');
    hold on

    
    body_x = [27 31];
    body_y = [11 14];

    plot(body_x,body_y, 'b','LineWidth', 2,'Color', 'black'); 
    hold on
    
    hand_x = [30 35];
    hand_y = [13 12];
    plot (hand_x,hand_y,'LineWidth', 2,'Color', 'black')
    plot(mx-k,my,'Color',[0.8500 0.3250 0.0980]);
    hold on
    plot(gx-k,gy);
    %fill(gx,gy,[0.8500 0.3250 0.0980])
    %hold on
    % plot the circular bit of the pedal
    plot(pedal_X, pedal_Y, 'k')
    hold on

    plot(back_pedal_X, back_pedal_Y, 'k')
    hold on

    plot(upper_bike_chain_X , upper_bike_chain_Y , 'k')
    hold on

    plot(lower_bike_chain_X, lower_bike_chain_Y, 'k')
    hold on

    plot(front_wheel_X, front_wheel_Y, 'k','LineWidth', 2)
    hold on

    plot(back_wheel_X, back_wheel_Y, 'k','LineWidth', 2)
    hold on

    plot(bike_chasis_X, bike_chasis_Y, 'k')
    hold on
    
    plot(steering_rod_X , steering_rod_Y , 'k', 'LineWidth', 2)
    hold on

    % plot the pedal crank/step
    pedal_crank_X = [32, outer_pedal_X(k)];
    pedal_crank_Y = [7, outer_pedal_Y(k)];
    plot(pedal_crank_X, pedal_crank_Y,'ko-', 'LineWidth', 2)


    % simulate front wheel movement
    front_wheel_spoke_1_X = [39, front_wheel_X(k)];
    front_wheel_spoke_1_Y = [7, front_wheel_Y(k)];
    plot(front_wheel_spoke_1_X, front_wheel_spoke_1_Y,'k-')

    % simulate back wheel movement
    back_wheel_spoke_1_X = [25, back_wheel_X(k)];
    back_wheel_spoke_1_Y = [7, back_wheel_Y(k)];
    plot(back_wheel_spoke_1_X, back_wheel_spoke_1_Y,'k-')
    
    leg_X =[27 32 outer_pedal_X(k)];
    leg_Y =[11 9 outer_pedal_Y(k)];
    plot(leg_X,leg_Y,'LineWidth', 2,'Color', 'black')
    hold on
    
    axis([0 50 0 50]);
    axis off equal

    % create a frame with each loop
    movieVector(k) = getframe(gcf);
    hold off
        continue;

end
for k = 1:n
    clf;
      head_x=32;
    head_y=14.5;
    r =1;
    th = 0:pi/100:2*pi;
    xunit = r * cos(th) + head_x;
    yunit = r * sin(th) + head_y;
    plot(xunit, yunit,'LineWidth', 2,'Color', 'black');
    hold on

    
    body_x = [27 31];
    body_y = [11 14];

    plot(body_x,body_y, 'b','LineWidth', 2,'Color', 'black'); 
    hold on
    
    hand_x = [30 35];
    hand_y = [13 12];
    plot (hand_x,hand_y,'LineWidth', 2,'Color', 'black')
    %road plot
    plot (roadx,roady);
    hold on
    % plot the circular bit of the pedal
    plot(pedal_X, pedal_Y, 'k')
    hold on

    plot(back_pedal_X, back_pedal_Y, 'k')
    hold on

    plot(upper_bike_chain_X , upper_bike_chain_Y , 'k')
    hold on

    plot(lower_bike_chain_X, lower_bike_chain_Y, 'k')
    hold on

    plot(front_wheel_X, front_wheel_Y, 'k','LineWidth', 2)
    hold on

    plot(back_wheel_X, back_wheel_Y, 'k','LineWidth', 2)
    hold on

    plot(bike_chasis_X, bike_chasis_Y, 'k')
    hold on
    
    plot(steering_rod_X , steering_rod_Y , 'k', 'LineWidth', 2)
    hold on

    % plot the pedal crank/step
    pedal_crank_X = [32, outer_pedal_X(k)];
    pedal_crank_Y = [7, outer_pedal_Y(k)];
    plot(pedal_crank_X, pedal_crank_Y,'ko-', 'LineWidth', 2)


    % simulate front wheel movement
    front_wheel_spoke_1_X = [39, front_wheel_X(k)];
    front_wheel_spoke_1_Y = [7, front_wheel_Y(k)];
    plot(front_wheel_spoke_1_X, front_wheel_spoke_1_Y,'k-')

    % simulate back wheel movement
    back_wheel_spoke_1_X = [25, back_wheel_X(k)];
    back_wheel_spoke_1_Y = [7, back_wheel_Y(k)];
    plot(back_wheel_spoke_1_X, back_wheel_spoke_1_Y,'k-')
    
     leg_X =[27 32 outer_pedal_X(k)];
    leg_Y =[11 9 outer_pedal_Y(k)];
    plot(leg_X,leg_Y,'LineWidth', 2,'Color', 'black')
    hold on
    
    axis([0 50 0 50]);
    axis off equal

    % create a frame with each loop
    movieVector(k+60) = getframe(gcf);
    hold off
        continue;

end
for k = 1:n
    clf;
      head_x=32;
    head_y=14.5;
    r =1;
    th = 0:pi/100:2*pi;
    xunit = r * cos(th) + head_x;
    yunit = r * sin(th) + head_y;
    plot(xunit, yunit,'LineWidth', 2,'Color', 'black');
    hold on

    
    body_x = [27 31];
    body_y = [11 14];

    plot(body_x,body_y, 'b','LineWidth', 2,'Color', 'black'); 
    hold on
    
    hand_x = [30 35];
    hand_y = [13 12];
    plot (hand_x,hand_y,'LineWidth', 2,'Color', 'black')
    %school plot
    plot (roof_X,roof_Y,'Color','black')
    fill(roof_X,roof_Y,[0.6350, 0.0780, 0.1840])
    hold on
    plot (school_X,school_Y,'Color','black')
    hold on
    plot (doors_X,doors_Y,'Color','black')
    hold on
    plot(w0X,w0Y,w1X,w1Y,w2X,w2Y, 'Color', 'black')
    hold on
    % plot the circular bit of the pedal
    plot(pedal_X, pedal_Y, 'k')
    hold on

    plot(back_pedal_X, back_pedal_Y, 'k')
    hold on

    plot(upper_bike_chain_X , upper_bike_chain_Y , 'k')
    hold on

    plot(lower_bike_chain_X, lower_bike_chain_Y, 'k')
    hold on

    plot(front_wheel_X, front_wheel_Y, 'k','LineWidth', 2)
    hold on

    plot(back_wheel_X, back_wheel_Y, 'k','LineWidth', 2)
    hold on

    plot(bike_chasis_X, bike_chasis_Y, 'k')
    hold on
    
    plot(steering_rod_X , steering_rod_Y , 'k', 'LineWidth', 2)
    hold on

    % pedal
    pedal_crank_X = [32, outer_pedal_X(k)];
    pedal_crank_Y = [7, outer_pedal_Y(k)];
    plot(pedal_crank_X, pedal_crank_Y,'ko-', 'LineWidth', 2)


    % front wheel movement
    front_wheel_spoke_1_X = [39, front_wheel_X(k)];
    front_wheel_spoke_1_Y = [7, front_wheel_Y(k)];
    plot(front_wheel_spoke_1_X, front_wheel_spoke_1_Y,'k-')

    % back wheel movement
    back_wheel_spoke_1_X = [25, back_wheel_X(k)];
    back_wheel_spoke_1_Y = [7, back_wheel_Y(k)];
    plot(back_wheel_spoke_1_X, back_wheel_spoke_1_Y,'k-')
     leg_X =[27 32 outer_pedal_X(k)];
    leg_Y =[11 9 outer_pedal_Y(k)];
    plot(leg_X,leg_Y,'LineWidth', 2,'Color', 'black')
    hold on
    
    axis([0 50 0 50]);
    axis off equal
    
    % create a frame with each loop
    movieVector(k+120) = getframe(gcf);
    hold on
    
        continue;

end

movie(movieVector, 1,40)

myWriter = VideoWriter('school','MPEG-4');
myWriter.FrameRate = 70;

open(myWriter)
writeVideo(myWriter, movieVector)
close(myWriter)