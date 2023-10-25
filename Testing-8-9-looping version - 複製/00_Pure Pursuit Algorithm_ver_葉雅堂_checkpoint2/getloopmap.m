function X_map = getloopmap()

load SpaFrancorchamps
% load Monza
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
end