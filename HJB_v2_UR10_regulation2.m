%RNN for robot manipulator UR10
clear all;
close all;
% rd=[0.55;0;1.3];
alpha=0.01;
eps=0.001;

%rd trajectory
dt=0.008;
rad=0.2;

Xmid=(-1.07-0.69)/2;
Ymid=(-0.27+0.345)/2;
Zmid=(0.1+0.05)/2;


th=0:0.2*dt:2*pi;
xd=rad*cos(th)+Xmid*ones(1,size(th,2));
yd=rad*sin(th)+Ymid*ones(1,size(th,2));
zd=Zmid*ones(1,size(th,2));
rd=[xd;yd;zd];
rd_reg=[-0.3;0.3;0.1];
n=length(th);
t=0:dt:dt*(n-2);
umin=-3; umax=3;

 d1 = 0.1273, d4 = 0.163941, d5 = 0.1157, d6 = 0.0922;
 a2 = -0.612, a3 = -0.5723;

I = [1 0 0;0 1 0;0 0 1];
% theta0=rand(6,1);
theta0=[-0.51;-1.05;1.49;-1.818;-1.452;-1.5937];


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




% t=1:dt:10
% n=length(t);

theta(:,1)=theta0;
r(:,1)=[x0;y0;z0];
w=rand(3,1);
v=0;

for k=1:n-1
    
%     J11=-sin(theta(1,k))*(a2*cos(theta(2,k))+a3*cos(theta(2,k)+theta(3,k)))-d3*cos(theta(1,k));
%     J12= cos(theta(1,k))*(-a2*sin(theta(2,k))-a3*sin(theta(2,k)+theta(3,k)));
%     J13=cos(theta(1,k))*(-a3*sin(theta(2,k)+theta(3,k)));
%     J21=cos(theta(1,k))*(a2*cos(theta(2,k))+a3*cos(theta(2,k)+theta(3,k)))-d3*sin(theta(1,k));
%     J22=sin(theta(1,k))*(-a2*sin(theta(2,k))-a3*sin(theta(2,k)+theta(3,k)));
%     J23=sin(theta(1,k))*(-a3*sin(theta(2,1)+theta(3,1)));
%     J31=0; 
%     J32= -a2*cos(theta(2,k))-a3*cos(theta(2,k)+theta(3,k));
%     J33=-a3*cos(theta(2,k)+theta(3,k));
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
    
    s1 = S(1,1); s2= S(2,2); s3= S(3,3);
    g1 = (s1^2+10*s1+2)/(s1^3+10*s1^2+2*s1+0.01);
    g2 = (s2^2+10*s2+2)/(s2^3+10*s2^2+2*s2+0.01);
    g3 = (s3^2+10*s3+2)/(s3^3+10*s3^2+2*s3+0.01);
    
    Ji = g1*V(:,1)*U(:,1)'+g2*V(:,2)*U(:,2)'+g3*V(:,3)*U(:,3)';
        
        
        
       for g= 1:6 
        gradJ(g) = (2*theta(g,k))/((3-theta(g,k))^2*(theta(g,k)+3)^2);
        if(gradJ >= 0)
            rm(g) = 1+gradJ(g);
        elseif(gradJ < 0)
            rm(g) = 1;
        end

        
       end
       
%        R = [rm(1)^(-0.5) 0 0 0 0 0;
%            0 rm(2)^(-0.5) 0 0 0 0;
%            0 0 rm(3)^(-0.5) 0 0 0;
%            0 0 0 rm(4)^(-0.5) 0 0;
%            0 0 0 0 rm(5)^(-0.5) 0;
%            0 0 0 0 0 rm(6)^(-0.5)];
    
        E = (rd_reg-r(:,k));
%         num =sqrt(E'*E)*R*J'*E;
        Q = [25 0 0;0 25 0;0 0 25];
    
        num = J'*E*sqrt(E'*Q*E);
        den = norm(J'*E);

%          up=num/(den*den)+ J'*w(:,k); % mine with steady state Jinv* rd dot estimate
%          up=num/(den)+ J'*w(:,k); % mine with steady state Jinv* rd dot RNN estimate
%                   up=(1-exp(-0.005*k))*(num/(den)+ Ji*((rd(:,k+1)-rd(:,k))/dt)); % mine with steady state Jinv* rd dot SVF estimate
                  up=(1-exp(-0.008*k))*(num/(den)); % mine with steady state Jinv* rd dot SVF estimate


%         for j=1:6
%         if up(j)<umin
%             u(j,k)=umin;
%         else if up(j)>umax
%                 u(j,k)=umax;
%             
%             else  u(j,k)=up(j);
%             end
%             end
%         end
%         
           for j=1:6
            u(j,k)=up(j);
           end
    
        
        theta(:,k+1)=theta(:,k)+dt*u(:,k);
        r(:,k+1)=r(:,k)+dt*J*u(:,k);
        w(:,k+1)= w(:,k)+ (1/eps)*(rd(:,k+1)-rd(:,k)) - (1/eps)*dt*(J*J'*w(:,k)); 

        e(:,k)=rd_reg-r(:,k+1);
              
        omega(:,k) =  e(:,k)'* e(:,k)+up'*up;
        v=omega(:,k)+v;
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
plot(t,u(4,:))
plot(t,u(5,:))
plot(t,u(6,:))
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

% figure;
% plot(t,r(1,2:end),'b.')
% hold on;
% plot(t,r(2,2:end),'b-')
% plot(t,r(3,2:end),'b--')
% plot(t,rd(1,2:end),'r.')
% plot(t,rd(2,2:end),'r-')
% plot(t,rd(3,2:end),'r--')
% title('Trajectories')
% xlabel('time')
% ylabel('r,rd')
% 

figure;
plot(t,r(1,2:end))
hold on;
plot(t,r(2,2:end))
plot(t,r(3,2:end))
plot(rd_reg(1),'r*')
plot(rd_reg(2),'r*')
plot(rd_reg(3),'r*')
title('Desired point and Trajectory')
xlabel('time')
ylabel('trajectory')

figure;
plot3(r(1,2:end),r(2,2:end),r(3,2:end))
hold on;
scatter3(rd_reg(1),rd_reg(2),rd_reg(3))
title('Desired point and Trajectory')
xlabel('time')
ylabel('trajectory')


figure;
plot(t,err')
title('RMS error')
xlabel('time')
ylabel('error')
figure;
plot(t,omega)
xlabel('time')
ylabel('cost')

