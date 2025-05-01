%% p1 a)
% infinite time horizon LQI optimal control design
clear all
A = 1;
B = 1;
C = 1;
D = 0;
Abar = [A 0;C 0];
Bbar = [B;0];
Cbar = [C 0];
Q = 10;
R = 1;
[Kxbar,S,CLP] = lqr(Abar,Bbar,Q,R);
Kx = Kxbar(1:2)
Kxi = Kxbar(end)

% p2 b)
sys=ss(Abar,Bbar,Kxbar,0);
% DG=tf(sys)
% margin(DG)
% allmargin(DG)
tf = 10;
[x, y, u, r,t]=LQI(Abar,Bbar,Cbar,D,2,Kxbar, 50);
plots_r(x, y, u, r,t,'p1 c')

function plots_r(x, y, u, r,t,title)
    figure; % Create a new figure
    subplot(3,1,1); 
    plot(t,y)
    hold on
    plot(t,r)
    ylabel('y');
    xlabel('t')
    legend('y','r')
    
    subplot(3,1,2);
    plot(t,x)
    ylabel('x');
    xlabel('t')
    legend('x1', 'x2', 'xi')
    
    subplot(3,1,3); 
    plot(t,u)
    ylabel('u');
    xlabel('t')
    sgtitle(title)
end

function [x, y, u, r,t]=LQI(A,B,C,D,ndim,Kx, tf)

    dt=0.01;
        
    t=0:dt:tf;
    
    n=length(t);
    
    x=zeros(ndim,n);
    
    u=zeros(1,n);
    
    r=ones(1,n)*3;
    v = zeros(ndim,n);
    v(end,:) = -r;
    
    Ainv=inv(eye(size(A))-dt*A/2);
    
    for i=1:n
    
         u(i)=-Kx*x(:,i);
    
         y(i)=C*x(:,i)+D*u(i);
    
         if i<n
    
              x(:,i+1)=Ainv*(x(:,i)+dt*(A*x(:,i)/2+B*u(i)+v(:,i)))+0.1*randn(1);
    
         end
    
    end
end


