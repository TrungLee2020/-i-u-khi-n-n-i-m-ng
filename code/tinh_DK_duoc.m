syms a b c d e ;
A1 = [0 1 0 1 0;
    1 0 1 0 1;
    0 1 0 1 0;
    1 0 1 0 1;
    0 1 0 1 0;];
D = [2 0 0 0 0;
    0 3 0 0 0;
    0 0 2 0 0;
    0 0 0 3 0;
    0 0 0 0 2;];
L = D - A1;
[v, e] = eig(L);
I = eye(5);
L - 2*I
B = [0.5;
    1;
    -1;];
A = [-1 2 0.5;
    3 1 -1.5;
    4 -2 1;];
co = ctrb(A, B);
rank(co);
%-> hệ có tính điều khiển được
% hệ có tính quan sát được, y = Cx -> ob = obsv(A, C)