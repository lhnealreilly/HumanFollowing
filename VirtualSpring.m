close all

stepLength = .5;
maxTurn = (pi/8)*stepLength;

num_steps = 50;

x = zeros(1, num_steps);
y = zeros(1, num_steps);
angle = zeros(1, num_steps);

start_angle = 0;

turning = 0;
turn_steps = 0;

max_turn_length = 4;

for i=2:num_steps
    if(~turning)
        turning = randi([-1, 1]);
        turn_steps = 0;
    elseif turn_steps >= max_turn_length
        turning = false;
    else
        start_angle = start_angle + (maxTurn.*rand(1))*turning;
        turn_steps = turn_steps + 1;
    end
    x(i) = x(i-1) + stepLength*sin(start_angle);
    y(i) = y(i-1) + stepLength*cos(start_angle);
    angle(i) = start_angle;
end

% x = [0,0,-0.193166424178272,-0.442850463532395,-0.713379236911158,-1.28461965907151,-1.85586008123186,-2.42710050339221,-2.99834092555257,-3.32657716830359,-3.43195543936034,-3.21896808647729,-2.68793490561362,-2.15690172474996,-1.62586854388629,-1.09466155172334,-0.311408318810820,0.597407067544809,1.59654157888677,2.59567609022873];
% y = [0,1,1.98116600663200,2.94949336888704,3.91220525869322,4.73298804365527,5.55377082861732,6.37455361357937,7.19533639854143,8.13993206280455,9.13436427265552,10.1114192288949,10.9587702546344,11.8061212803739,12.6534723061134,13.5007143798310,14.1224171811212,14.5396156872586,14.5812116996331,14.6228077120076];
% angle = [0,0,-0.194388332909179,-0.252353946130606,-0.273942243137837,-0.608016329120596,-0.608016329120596,-0.608016329120596,-0.608016329120596,-0.334435759212937,-0.105574282366072,0.214631453800812,0.559819406936533,0.559819406936533,0.559819406936533,0.560024543259508,0.899881483216960,1.14043577326276,1.52918830997208,1.52918830997208];

offset_dist = 1;
offset_angle = pi/2;

pred_human_x = x;
pred_human_y = y;

delta_angle = 0;

for i=2:num_steps
    delta_angle = angle(i) - angle(i-1);
    pred_human_x(i) = x(i-1) + stepLength * sin(angle(i) + delta_angle);
    pred_human_y(i) = y(i-1) + stepLength * cos(angle(i) + delta_angle);
end
    
desired_robot_X = pred_human_x + (offset_dist * sin(offset_angle + angle));
desired_robot_Y = pred_human_y + (offset_dist * cos(offset_angle + angle));

obstacle_x = desired_robot_X(9);
obstacle_y = desired_robot_Y(9);
obstacle_spring_k = -.2;

robot_real_x = zeros(1, num_steps);
robot_real_y = zeros(1, num_steps);

robot_real_x(1) = desired_robot_X(1) + 1.5;
robot_real_y(1) = desired_robot_Y(1) + 1;

primary_spring_k = .5;
secondary_spring_k = .5;

%Virtual Spring Calculator:
for i=1:num_steps-1

    
    spring1_x = primary_spring_k * (desired_robot_X(i) - robot_real_x(i));
    spring1_y = primary_spring_k * (desired_robot_Y(i) - robot_real_y(i));
    spring2_x = secondary_spring_k * (desired_robot_X(i+1) - robot_real_x(i));
    spring2_y = secondary_spring_k * (desired_robot_Y(i+1) - robot_real_y(i));
        
    step_limiter = min(stepLength, abs(1/hypot(spring1_x + spring2_x + obstacle_spring_x, spring1_y + spring2_y + obstacle_spring_y)));
    
    robot_real_x(i+1) = robot_real_x(i) + (spring1_x + spring2_x) * step_limiter;
    robot_real_y(i+1) = robot_real_y(i) + (spring1_y + spring2_y) * step_limiter;
    
    obstacle_spring_x = obstacle_spring_k * ((obstacle_x - robot_real_x(i+1))/(hypot(obstacle_x-robot_real_x(i+1), obstacle_y-robot_real_y(i+1)))^2);
    obstacle_spring_y = obstacle_spring_k * ((obstacle_y - robot_real_y(i+1))/(hypot(obstacle_x-robot_real_x(i+1), obstacle_y-robot_real_y(i+1)))^2);
    
    robot_real_x(i+1) = robot_real_x(i) + (spring1_x + spring2_x + obstacle_spring_x) * step_limiter;
    robot_real_y(i+1) = robot_real_y(i) + (spring1_y + spring2_y + obstacle_spring_y) * step_limiter;
end


figure
hold on
grid on
plot(x, y);
plot(pred_human_x, pred_human_y);
plot(desired_robot_X, desired_robot_Y);
plot(robot_real_x, robot_real_y);
plot(obstacle_x, obstacle_y, 'd');
legend('Human', 'Human Predicted', 'Robot Desired', 'Robot Actual')
hold off

diff = hypot(desired_robot_X-robot_real_x, desired_robot_Y-robot_real_y);

figure
hold on
grid on
plot(diff);
legend(string(mean(diff)));
hold off
