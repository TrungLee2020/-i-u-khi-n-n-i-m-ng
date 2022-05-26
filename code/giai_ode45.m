%Created by Anokina Shalimoon
%CTC_Undirected.m
%This function implements continuous time consensus algorithm for
%undirected graph which is an example applied to a four nodes case
%This Function calls CTC.m
clear; close all;
A_undir = ones(4)-eye(4); %adjacency Matrix
D_undir = diag(sum(A_undir,1)); %Degree Matrix (Out-Degree)
L_undir = D_undir - A_undir; %Laplacian Matrix
[V,D,W] = eig(L_undir); %calculating right eigenvectors, V, the eigenvalues,D, and the left eigenvectors, W
eig_dir = round(diag(D),4);
    lambda_1 = eig_dir(1)
    lambda_2 = eig_dir(2)
    lambda_3 = eig_dir(3)
    lambda_4 = eig_dir(4)
w1 = W(:,1); %Normalized left eigenvector of Laplacian L for lambda_1 = 0
tau = 1/lambda_2; %time constant based on lambda_2
%initial and final time
t_initial = 0;
t_final = 10;
x_initial = [1 2 3 4]'; %initial states
% calculation of consensus value c
c_dir = w1'*x_initial;
evalue = eig(L_undir);
%calculating ode45 to compute state x
[t,x] = ode45(@(t,x) CTC(t,x,L_undir),[t_initial t_final],x_initial);
% plot figures
figure(1)
plot(graph(A_undir))
title('Undirected Graph Representation')
figure(2)
plot(t,x)
legend('Node 1','Node 2','Node 3','Node 4')
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