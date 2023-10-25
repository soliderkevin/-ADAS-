clear;
load monza.mat
TX = [X Y];
[L2,R2,K2] = curvature(TX);
figure;
plot(L2,R2)
title('Curvature radius vs. cumulative curve length')
xlabel L
ylabel R
figure;
h = plot(X,Y); grid on; axis equal
set(h,'marker','.');
xlabel x
ylabel y
title('2D curve with curvature vectors')
hold on
quiver(X,Y,K2(:,1),K2(:,2));
hold off