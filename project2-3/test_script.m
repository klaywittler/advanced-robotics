% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% ekf_handle1 = @(sensor, vic) ekf1(sensor, vic, your_input);
% ekf_handle2 = @(sensor) ekf2(sensor, your_input);
%
% We will only call ekf_handle in the test function.
% Note that this will only create a function handle, but not run the function
close all; clear all;
addpath('submission')
load('data/studentdata1.mat');
load('aprilTagMap.mat');

Kinv = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1]\eye(3);
T = [-0.04; 0; -0.03];
R = eulzxy2rot([pi,0,-44.5*pi/180]);

% estimate_pose_handle = @(sensor) estimate_pose(sensor,Kinv, pA, R, T);
% estimate_vel_handle = @(sensor) estimate_state(sensor, Kinv, pA, R, T);
ekf1_handle = @(sensor, vic) ekf1(sensor, vic, Kinv, pA, R, T);
ekf2_handle = @(sensor) ekf2(sensor, Kinv, pA, R, T);

ekf = 1;

qVicon = zeros(4,numel(time));
for i=1:numel(time)
    qVicon(:,i) = eulzxy2quat([vicon(4,i),vicon(5,i),vicon(6,i)]);
end

if ekf == 1
    X1 = zeros(7,numel(vicon(1,:)));
    Z1 = zeros(9,numel(vicon(1,:)));
    j = 1;
    elapsedTime = zeros(1, numel(vicon(1,:)));
    profile on
    for i=1:numel(vicon(1,:))
        if time(i) == data(j).t
            d = data(j);
            if j < numel(data)
                j = j+1;
            end
        else
            d.id = [];
            d.is_ready = 0;
        end
        v.vel = vicon(7:12,i);
        v.t = time(i);
        tic
        [x, Z1(:,i)] = ekf1_handle(d, v);
        
        elapsedTime(i) = toc;
        if ~isempty(x)
          X1(:,i)  = x;
        end
    end 
    profile report
    profile off
    
    est_time = time;
    est_data = X1;
    ground_time = time;
    ground_data = [vicon(1:3,:);  qVicon];
    
    plotting(time,X1,time,ground_data)

elseif ekf == 2
    warning('off')
    X2 = zeros(10,numel(data));
    Z2 = zeros(15,numel(data));
    t = [data.t];
    elapsedTime = zeros(1, numel(data));
    profile on
    for i=1:numel(data)
        tic
        [x, Z2(:,i)] = ekf2_handle(data(i));
        elapsedTime(i) = toc;
        if ~isempty(x)
          X2(:,i)  = x;
        end 
    end  
    profile report
    profile off

    est_time = t;
    est_data = X2;
    ground_time = time;
    ground_data = [vicon(1:3,:); vicon(7:9,:);  qVicon];
    
    plotting(t,X2,time,ground_data)  
end

[rms_err, ~] = calc_err(ground_time', ground_data', est_time', est_data');
disp(['rms error: ',  num2str(rms_err)]); 

disp(['Average run time (ms): ',  num2str(1000*mean(elapsedTime))]); 

function plotting(t,X,time,vicon)
    if numel(X(:,1)) > 7
        c = 3;
        n = 3;
        p = true;
    else
        c = 2;
        n = 0;
        p = false;
    end
    figure()
    %%% x1 plots
    subplot(4,c, 1)
    plot(t,X(1,:),time,vicon(1,:))
    xlabel('t')
    ylabel('x [m]')
    title('position')
    subplot(4,c,1+c)
    plot(t,X(2,:),time,vicon(2,:))
    xlabel('t')
    ylabel('y [m]')
    subplot(4,c,1+2*c)
    plot(t,X(3,:),time,vicon(3,:))
    xlabel('t')
    ylabel('z [m]')
    %%% legend
    hSub = subplot(4,c,1+3*c); plot(1,1,1,1);
    hLegend = legend('estimated','vicon');
    set(hLegend, 'position', get(hSub, 'position')); % Adjusting legend's position     
    axis(hSub,'off');           % Turning its axis off
    %%% orientation plots
    subplot(4,c, 2)
    plot(t,X(4+n,:),time,vicon(4+n,:))
    xlabel('t')
    ylabel('w')
    title('orientation')
    subplot(4,c,2 + c)
    plot(t,X(5+n,:),time,vicon(5+n,:))
    xlabel('t')
    ylabel('qx')
    subplot(4,c, 2 + 2*c)
    plot(t,X(6+n,:),time,vicon(6+n,:))
    xlabel('t')
    ylabel('qy')
    subplot(4,c, 2 + 3*c)
    plot(t,X(7+n,:),time,vicon(7+n,:))
    xlabel('t')
    ylabel('qz')
    if p
        %%% velocity plots
        subplot(4,c, 3)
        plot(t,X(4,:),time,vicon(4,:))
        xlabel('t')
        ylabel('vx [m/s]')
        title('linear velocity')
        subplot(4,c,3+c)
        plot(t,X(5,:),time,vicon(5,:))
        xlabel('t')
        ylabel('vy [m/s]')
        subplot(4,c, 3+2*c)
        plot(t,X(6,:),time,vicon(6,:))
        xlabel('t')
        ylabel('vz [m/s]') 
    end
end

function [rms_err, rms_err_ind] = calc_err(ground_time, ground_data, est_time, est_data)
    int_ground_data = interp1(ground_time, ground_data, est_time);
    rms_err_ind = sqrt(sum((int_ground_data-est_data).^2,1)./numel(est_time));
    rms_err = sqrt(sum((vecnorm(int_ground_data,2,2)-vecnorm(est_data,2,2)).^2,1)./numel(est_time));
%     j = 1;
%     rms_err_ind = 0;
%     err = zeros(numel(est_time),1);
%      for i=1:numel(ground_data(1,:))
%         if ground_time(i) == est_time(j)
%             err(j) = norm(ground_data(i)-est_data(j));
%             if j < numel(est_data)
%                 j = j+1;
%             end
%         end
%      end   
%     rms_err = sqrt(sum(err.^2))/numel(est_time);
end