%Created by Anokina Shalimoon
%CTC_Undirected.m
%This function implements continuous time consensus algorithm for
%undirected graph which is an example applied to a four nodes case
%This Function calls CTC.m
clear; close all;
A_undir = [0 1 1 0 0;1 0 1 1 0;1 1 0 0 0;0 1 0 0 1;0 0 0 1 0;]; %adjacency Matrix
D_undir = [2 0 0 0 0;0 3 0 0 0;0 0 2 0 0;0 0 0 2 0; 0 0 0 0 1]; %Degree Matrix (Out-Degree)
L_undir = D_undir - A_undir %Laplacian Matrix
[V,D,W] = eig(L_undir); %calculating right eigenvectors, V, the eigenvalues,D, and the left eigenvectors, W
eig_dir = round(diag(D),5);
    lambda_1 = eig_dir(1);
    lambda_2 = eig_dir(2);
    lambda_3 = eig_dir(3);
    lambda_4 = eig_dir(4);
    lambda_5 = eig_dir(5);
w1 = W(:,1); %Normalized left eigenvector of Laplacian L for lambda_1 = 0
tau = 1/lambda_2; %time constant based on lambda_2
%initial and final time
t_initial = 0;
t_final = 10;
pos = 10*(rand(5,1)-0.5); % random positions start
%x_initial = [2 4 -3 -5 0]'; %initial states
% calculation of consensus value c: giá trị đồng thuận c
%c_dir = w1'*x_initial;
evalue = eig(L_undir);
%calculating ode45 to compute state x
[t,x] = ode45(@(t,x) CTC(t,x,L_undir),[t_initial t_final],pos);
y = x';
% plot figures
figure(1)
plot(graph(A_undir))
title('Undirected Graph Representation')
figure(2)
plot(t,y)
legend('Node 1','Node 2','Node 3','Node 4', 'Node 5')
title('Discrete-time consensus for Undirected Graph')
xlabel({'t','(in seconds)'})
ylabel('x')
figure(3)
plot(evalue,0,'r*','LineWidth',1)
title('Eigenvalues of the Laplacian Matrix For Undirected Graph')
xlabel('real')
ylabel('imag')
axis equal;
grid on