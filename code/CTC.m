%CTC.m
%This is called by ODE45 in both CTC_Directed.m and CTC_Undirected. m
%to solve the xdot = -Lx differential equations.
function xdot = CTC(t,x,L)
	% b la ma tran 5x1
	%b = tong xichma theo hang nhan với kp
    xdot=-35*L*x + b;
end
