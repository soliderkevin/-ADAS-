function [Adj_Velocity] =  Velocity_Adjustment(curvature,velocity)

i = 1; % for looping there

looping = X_map(X);

while (i <= looping)
    data = X_map(:,1:4) %curvature_input
    data_average = X_map(:,i:i+3);

    curve=mean(data_average);

    if( curve >= 50)

        required_SD = std(data);

        disp(std_dev);
    else

        required_SD = mean(data);

    end
    i = i+1;
end


end

%速度psuedo code.
%以standard deviation 超過20為標準給定速
%mean 為平均速度function
%std 為標準差function