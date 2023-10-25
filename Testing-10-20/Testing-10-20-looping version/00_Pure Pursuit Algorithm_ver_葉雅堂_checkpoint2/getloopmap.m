function X_map = getloopmap()

    %loading Monza
     load Monza
[m,~]=size(X);
DT=0.1;
k=1;
path_length(1)=0;
    for i=2:m
        temp_y=Y(i,1)-Y(i-1,1); %
        temp_x=X(i,1)-X(i-1,1);
        temp_phi=atan2(temp_y,temp_x);
        phi(k)=mod(temp_phi,2*pi); %heading angle
        temp_s=temp_x^2+temp_y^2;
        s_arc(k,1)=sqrt(temp_s);
        path_length(i)=path_length(i-1)+s_arc(k,1);
        k=k+1;
    end
remainingdist=flip(path_length);
X_map(:,1)=X(2:end,1)'; 
X_map(:,2)=Y(2:end,1)'; 
X_map(:,3)=phi';
X_map(:,4)=remainingdist(1:end-1)';
X_map(:,5)=zeros(m-1,1); % altitude


%錯誤部分 Incorrect number or types of inputs or outputs for function 'circumcenter'
    TX = [X,Y];  %error point[L2,R2,K2] = curvature(TX);
    [L2,R2,K2] = curvature(TX)
    m=size(R2);
    phi = 0.7;


    for i = 1:m   % Formula for max_velocity in every point
        vel_max(i,1) = sqrt(phi*9.8*R2(i));
    end

X_map(:,6) = vel_max(2:end)'; 
% 
%     for i = 1:(X_map(:,6)-2)
%         X_map(:,5) = std((X_map(i,6))+(X_map(i+3,6)))
%     end

dt_sample_value = 150; %data samples

%X(i:map) fifth slot is the height
    
    for i = 1:length(X_map(:,6))
        if X_map(i,6)>30 %m/s -> k/hr
            X_map(i,6) = 30;
        end
    end

    for i = dt_sample_value:length(X_map(:,6))-dt_sample_value
        X_map(i:6) = mean(X_map(i:i+dt_sample_value,6));
        display(std(X_map(i:i+dt_sample_value,6)));
    end
    
    %Fixing the last point not getting calculated(Last row = First row)
    X_map(length(X_map(:,6)),6) = X_map(1,6);

    
  


%     X_map(i:6) = mean(X_map(i:i+dt_sample_value,6));
%     if std(X_map(i:i+dt_sample_value,6) >= 100) %when std is smaller than certaina mount, go pave
% %         if std(X_map(i:i+dt_sample_value,6)) <= 90 %bigger std more pave, between 70~80 will get that number, use 300 to pave the way, the gap between 70~80
% %     
% %             X_map(i:i+dt_sample_value,6) = X_map(i:i,6);
% %         else
% %             X_map(i,6) = mean(X_map(i:i+dt_sample_value,6));
% %         end  
%         X_map(i:i+dt_sample_value,6) = mean(X_map(i:i+dt_sample_value,6));
%     else
%         X_map(i:i+dt_sample_value,6) = 100;
%     end


%Map_sizing => 
%Data_sampling





end