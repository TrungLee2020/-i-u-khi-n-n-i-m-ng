%%
pos = randi(5,6,2); % random positions for starting of the agents
% initial formation of a hexagon centered on origin
final = [[0, 0.5];
    [-0.5,0];
    [0.5, 0];
    [-1, -0.5];
    [1,-0.5];
    [0, -0.5]];
% matrix where we store our positions during the loop for plotting
pos1 = [];
pos2 = [];
pos3 = [];
pos4 = [];
pos5 = [];
pos6 = [];
% matrix where we store the forces experienced by each agents for plotting
force_collect = [];
% temporary force matrix to be used for every agents
force_temp = zeros(1,2);
% activation radius for our force of repulsion
rs = 3.0; % kich thuoc vung an toan duoc chon
% time chunk that we are simulating to be going through
dt = 0.03; % 33 hz.
kp = 0.85; % formation_control_gain
kc = 1.5; % formation_movement_gain
N = 6; % number of agents
alpha = 0.5; % replusion_gain :100
beta = 0.3; % repulsion_ramp_gain: 2
plot(final(:,1),final(:,2),'--r*'); % plotting our final destination/formation goal
hold on; % giữ mọi thứ trên đồ thị
for t=0:dt:10 % 0 seconds to 10 seconds 0,0.03,0.06....10
    dv=zeros(6,2); % ma trận vận tốc cho mọi trường hợp
    force=zeros(6,2); % ma trận vector lực cho mọi trường hợp
    for i=1:1:N
        dv_temp = [0.0,0.0]; % temporary velocity variable
        for j=1:1:N % each agent
            dv_temp = dv_temp + pos(j,:)-pos(i,:)-(final(j,:)-final(i,:)); % Vận tốc kiểm soát đội hình tác tử
            % force equation
            if norm(pos(j,:)-pos(i,:)) > 0 && norm(pos(j,:) - pos(i,:)) < rs % Tránh va chạm giữa các tác tử
                force_repel = alpha*(exp(-beta*norm(pos(j,:)-pos(i,:))) - exp(-beta*rs));  % Lực vô hướng
                force_temp = force_temp + (pos(j,:)-pos(i,:))/norm(pos(j,:)-pos(i,:))*force_repel; % Vector lực
            else
                force_temp = [0,0];
            end
        end
        dv(i,1) = dv_temp(1); % vận tốc x
        dv(i,2) = dv_temp(2); % vận tốc y
        force(i,:) = force_temp;
        force_collect = [force_collect; force_temp];
    end
    % Xác định tâm của đội hình các tác tử
    centroid = zeros(1,2);
    for i=1:1:6
        centroid = centroid + pos(i,:);
    end
	%tracking control
    centroid = centroid/6;
    for i=1:1:6
        pos(i,1) = pos(i,1) + (kp*dv(i,1) - centroid(1,1)*kc - force(i,1))*dt;
        pos(i,2) = pos(i,2) + (kp*dv(i,2) - centroid(1,2)*kc - force(i,2))*dt;
    end
    pos1 = [pos1;pos(1,:)];
    pos2 = [pos2;pos(2,:)];
    pos3 = [pos3;pos(3,:)];
    pos4 = [pos4;pos(4,:)];
    pos5 = [pos5;pos(5,:)];
    pos6 = [pos6;pos(6,:)];
    plot(pos(1,1),pos(1,2),'r.');
    hold on;
    plot(pos(2,1),pos(2,2),'b.');
    hold on;
    plot(pos(3,1),pos(3,2),'g.');
    hold on;
    plot(pos(4,1),pos(4,2),'k.');
    hold on;
    plot(pos(5,1),pos(5,2),'m.');
    hold on;
    plot(pos(6,1),pos(6,2),'c.');
    hold on;
    pause(dt);
end
L_undir = [2 -1 -1 0 0;-1 3 -1 -1 0;-1 -1 2 0 0;0 -1 0 2 -1;0 0 0 -1 1;];
[V,D,W] = eig(L_undir); %calculating right eigenvectors, V, the eigenvalues,D, and the left eigenvectors, W
eig_dir = round(diag(D),5);
    lambda_1 = eig_dir(1)
    lambda_2 = eig_dir(2)
    lambda_3 = eig_dir(3)
    lambda_4 = eig_dir(4)
    lambda_5 = eig_dir(5)
w1 = W(:,1); %Normalized left eigenvector of Laplacian L for lambda_1 = 0
tau = 1/lambda_2; %time constant based on lambda_2
%initial and final time
t_initial = 0;
t_final = 10;
% calculation of consensus value c
%c_dir = w1'*pos;
evalue = eig(L_undir);
%calculating ode45 to compute state x
[t,x] = ode45(@(t,x) CTC(t,x,L_undir),[t_initial t_final],pos);
% plot figures
figure(3)
plot(graph(A_undir))
title('Undirected Graph Representation')
figure(4)
plot(t,x)
legend('Node 1','Node 2','Node 3','Node 4')
title('Discrete-time consensus for Undirected Graph')
xlabel({'t','(in seconds)'})
ylabel('x')
figure(5)
plot(evalue,0,'r*','LineWidth',1)
title('Eigenvalues of the Laplacian Matrix For Undirected Graph')
xlabel('real')
ylabel('imag')
axis equal;
grid on