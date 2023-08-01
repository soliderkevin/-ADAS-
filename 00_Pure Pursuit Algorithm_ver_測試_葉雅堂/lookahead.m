function [lookheading_point, closest_index] = lookahead4(velocity, current_Position, givenpath)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global DT
desired_HeadingDist = (1/6)*velocity^2*DT+(1/5)*velocity*DT+5;
% desired_HeadingDist = 20;
%% since fixed path length interval ,we can use index number to predict path length
closest_index = map_location(current_Position, givenpath);
lookheading_point = map_desired(closest_index, givenpath, desired_HeadingDist, current_Position);

end

function desired_vector = map_desired(closest_index, givenpath, desired_HeadingDist, current_Position)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
N=size(givenpath);
Temp1=givenpath(closest_index+1:end,1:2);
Temp2=givenpath(1:closest_index,1:2);
Temp= [Temp1; Temp2];
location_check = Temp(:,1:2)-current_Position(1,1:2);

for i=1:N
      candicate_vector=location_check(i,:);
      location_norm = norm(candicate_vector,2);
      Temp_candicate_HD  = location_norm; 
      candicate_HD  = abs(Temp_candicate_HD);
      if candicate_HD>=desired_HeadingDist
           desired_vector=candicate_vector+current_Position(1,1:2);
          break
      end
      
end

end

function [closest_index, closest_point] = map_location(current_Position, givenpath)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
location_check = givenpath(:,1:2)-current_Position(1,1:2);
[N,~] = size(location_check);

for i=1:N
      location_norm(i) = norm(location_check(i,:),2);
      Temp_candicate_HD  = location_norm(i); 
      candicate_HD (i) = abs(Temp_candicate_HD);
end
      [~,closest_index] = min(candicate_HD);
      closest_point=givenpath(closest_index,:);
end
