%RNN for robot manipulator UR10
clear all
close all
rd=[-0.3;0.3;0.1];
alpha=0.01;

umin=-5; umax=5;
% a2=0.4318; a3=0.0203; d1=0; d3=0.1245; d4=a2;

d1 = 0.1273, d4 = 0.163941, d5 = 0.1157, d6 = 0.0922;
a2 = -0.612, a3 = -0.5723;


theta0=rand(6,1);
% t0=a2*cos(theta0(2,1))+a3*cos(theta0(2,1)+theta0(3,1))-d4*sin(theta0(2,1)+theta0(3,1));
% x0=cos(theta0(1,1))*t0-d3*sin(theta0(1,1));
% y0=sin(theta0(1,1))*t0+d3*cos(theta0(1,1));
% z0=-a2*sin(theta0(2,1))-a3*sin(theta0(2,1)+theta0(3,1))-d4*cos(theta0(2,1)+theta0(3,1));




x0 = (sin(theta0(1,1))*cos(theta0(5,1))-((-cos(theta0(1,1))*cos(theta0(2,1))*sin(theta0(3,1))-cos(theta0(1,1))*sin(theta0(2,1))*cos(theta0(3,1)))*sin(theta0(4,1))+...
    (cos(theta0(1,1))*cos(theta0(2,1))*cos(theta0(3,1))-cos(theta0(1,1))*sin(theta0(2,1))*sin(theta0(3,1)))*cos(theta0(4,1)))*sin(theta0(5,1)))*d6+(...
    (cos(theta0(1,1))*cos(theta0(2,1))*cos(theta0(3,1))-cos(theta0(1,1))*sin(theta0(2,1))*sin(theta0(3,1)))*sin(theta0(4,1))-...
    (-cos(theta0(1,1))*cos(theta0(2,1))*sin(theta0(3,1))-cos(theta0(1,1))*sin(theta0(2,1))*cos(theta0(3,1)))*cos(theta0(4,1)))*d5+sin(theta0(1,1))*d4-cos(theta0(1,1))*...
    sin(theta0(2,1))*sin(theta0(3,1))*a3+cos(theta0(1,1))*cos(theta0(2,1))*cos(theta0(3,1))*a3+cos(theta0(1,1))*cos(theta0(2,1))*a2;

y0 = (-((-sin(theta0(1,1))*cos(theta0(2,1))*sin(theta0(3,1))-sin(theta0(1,1))*sin(theta0(2,1))*cos(theta0(3,1)))*sin(theta0(4,1))+...
    (sin(theta0(1,1))*cos(theta0(2,1))*cos(theta0(3,1))-sin(theta0(1,1))*sin(theta0(2,1))*sin(theta0(3,1)))*cos(theta0(4,1)))*sin(theta0(5,1))-cos(theta0(1,1))*cos(theta0(5,1)))*d6+(...
    (sin(theta0(1,1))*cos(theta0(2,1))*cos(theta0(3,1))-sin(theta0(1,1))*sin(theta0(2,1))*sin(theta0(3,1)))*sin(theta0(4,1))-...
    (-sin(theta0(1,1))*cos(theta0(2,1))*sin(theta0(3,1))-sin(theta0(1,1))*sin(theta0(2,1))*cos(theta0(3,1)))*cos(theta0(4,1)))*d5-cos(theta0(1,1))*d4-sin(theta0(1,1))*sin(theta0(2,1))...
    *sin(theta0(3,1))*a3+sin(theta0(1,1))*cos(theta0(2,1))*cos(theta0(3,1))*a3+sin(theta0(1,1))*cos(theta0(2,1))*a2;

z0 = -...
    ((cos(theta0(2,1))*cos(theta0(3,1))-sin(theta0(2,1))*sin(theta0(3,1)))*sin(theta0(4,1))+(cos(theta0(2,1))*sin(theta0(3,1))+sin(theta0(2,1))*cos(theta0(3,1)))*cos(theta0(4,1)))*...
    sin(theta0(5,1))*d6+...
    ((cos(theta0(2,1))*sin(theta0(3,1))+sin(theta0(2,1))*cos(theta0(3,1)))*sin(theta0(4,1))-(cos(theta0(2,1))*cos(theta0(3,1))-sin(theta0(2,1))*sin(theta0(3,1)))*cos(theta0(4,1)))*d5...
    +d1+cos(theta0(2,1))*sin(theta0(3,1))*a3+sin(theta0(2,1))*cos(theta0(3,1))*a3+sin(theta0(2,1))*a2;






dt=0.001;
t=1:dt:20;
n=length(t);

theta(:,1)=theta0;
r(:,1)=[x0;y0;z0];
w=rand(3,1);


for k=1:n
    J11 = (cos(theta(1,k))*cos(theta(5,k))-...
        ((sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))+sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*sin(theta(5,k)))*d6+...
        ((sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))-(sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))+sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5+cos(theta(1,k))*d4+...
        sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))*a3-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))*a3-sin(theta(1,k))*cos(theta(2,k))*a2;
    
    
    
    J12 = -((cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*...
        sin(theta(5,k))*d6+((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))-(cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-...
        cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))*a3-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k))*a3-cos(theta(1,k))*sin(theta(2,k))*a2;
    
    
    
    J13 = -((cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*...
        sin(theta(5,k))*d6+((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))-(cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-...
        cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))*a3-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k))*a3;
    
    
    J14 = ((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*d5-...
        ((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k))-(cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k)))*sin(theta(5,k))*d6;
    
    J15 = (-sin(theta(1,k))*sin(theta(5,k))-...
        ((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*cos(theta(5,k)))*d6;
    
    J16 = 0.0;
    
    
    
    J21 = (sin(theta(1,k))*cos(theta(5,k))-...
        ((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*sin(theta(5,k)))*d6+...
        ((cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k))-(-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5+sin(theta(1,k))*d4-...
        cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))*a3+cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))*a3+cos(theta(1,k))*cos(theta(2,k))*a2;
    
    J22= -((sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*...
        sin(theta(5,k))*d6+((-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))-(sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-...
        sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))*a3-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k))*a3-sin(theta(1,k))*sin(theta(2,k))*a2;
    
    J23 =  -((sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*...
        sin(theta(5,k))*d6+((-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))-(sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-...
        sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))*a3-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k))*a3;
    
    J24= ((-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*d5-...
        ((-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k))-(sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k)))*sin(theta(5,k))*d6;
    
    
    J25= (cos(theta(1,k))*sin(theta(5,k))-...
        ((-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*cos(theta(5,k)))*d6;
    
    J26 = 0;
    
    
    
    J31 = 0.0;
    
    J32 =  -((-cos(theta(2,k))*sin(theta(3,k))-sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*sin(theta(5,k))*d6+...
        ((cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k))-(-cos(theta(2,k))*sin(theta(3,k))-sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-sin(theta(2,k))*sin(theta(3,k))*a3+cos(theta(2,k))*cos(theta(3,k))*a3+...
        cos(theta(2,k))*a2;
    
    J33 = -((-cos(theta(2,k))*sin(theta(3,k))-sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*sin(theta(5,k))*d6+...
        ((cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k))-(-cos(theta(2,k))*sin(theta(3,k))-sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-sin(theta(2,k))*sin(theta(3,k))*a3+cos(theta(2,k))*cos(theta(3,k))*a3;
    
    J34 = ((cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k))+(cos(theta(2,k))*sin(theta(3,k))+sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-...
        ((cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k))-(cos(theta(2,k))*sin(theta(3,k))+sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k)))*sin(theta(5,k))*d6;
    
    J35 = -((cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k))+(cos(theta(2,k))*sin(theta(3,k))+sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*cos(theta(5,k))*d6;
    
    J36 = 0.0;
    
    
    J=[J11 J12 J13 J14 J15 J16 ; J21 J22 J23 J24 J25 J26 ; J31 J32 J33 J34 J35 J36];
    
     [U,S,V] = svd(J);
    
    
    
%     Jinv = pinv(J);
      R = [1 0 0;0 1 0;0 0 1];
    
%     Rinv = pinv(R);
%     [U,S,V] = svd(Rinv);
%     SC = [sqrt(S(1))+0.001 0 0;0 sqrt(S(2))+0.001 0; 0 0 sqrt(S(3))+0.001];
%     SCinv= pinv(SC);
%     C= U*SCinv*V;


    E = rd-r(:,k);
    
    num = 1*J'*E*sqrt(E'*E);
    den = norm(J'*E);


    up= num/(den*den);
    %      up= J'*Rinv*C*(rd-r(:,k));
    
    %    up=J'*w(:,k);
    
%     for j=1:6
%         if up(j)<umin
%             u(j,k)=umin;
%         else if up(j)>umax
%                 u(j,k)=umax;
%                 
%             else  u(j,k)=up(j);
%             end
%         end
%     end
    
    
           for j=1:6
            u(j,k)=up(j);
           end
    
    
    theta(:,k+1)=theta(:,k)+dt*u(:,k);
    r(:,k+1)=r(:,k)+dt*J*u(:,k);
%     w(:,k+1)= w(:,k)+ (1/eps)*(Rinv*C*(rd-r(:,k+1)))*dt - (1/eps)*dt*(J*J'*w(:,k));
    e(:,k)=rd-r(:,k+1);
%     d(:,k) = eigs(J*J',3);
    err(k,1)=sqrt(sum(e(:,k).^2)/3);
end

figure;
plot(t,theta(1,2:end))
hold on;
plot(t,theta(2,2:end))
plot(t,theta(3,2:end))
plot(t,theta(4,2:end))
plot(t,theta(5,2:end))
plot(t,theta(6,2:end))
title('Time history of Joint Angle')
xlabel('time')
ylabel('joint angles')

figure;
plot(t,u(1,:))
hold on;
plot(t,u(2,:))
plot(t,u(3,:))
title('time history of Control Input')
xlabel('time')
ylabel('u')

figure;
plot(t,e(1,:))
hold on;
plot(t,e(2,:))
plot(t,e(3,:))
title('error trajectories')
xlabel('time')
ylabel('error')

figure;
plot(t,r(1,2:end))
hold on;
plot(t,r(2,2:end))
plot(t,r(3,2:end))
plot(rd(1),'r*')
plot(rd(2),'r*')
plot(rd(3),'r*')
title('Desired point and Trajectory')
xlabel('time')
ylabel('trajectory')


figure()
