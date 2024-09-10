% Email: mraza.engg@gmail.com

clc;
close all,
format short;
tic
m=load('loaddata33bus.m');
l=load('linedata33bus.m');

% m=load('loaddata69bus.m');
% l=load('linedata69bus.m');
PL_Cmp=10^7;
br=length(l);
no=length(m);
f=0;
d=0;
MVAb=100;
KVb=12.66;
Zb=(KVb^2)/MVAb;
% Per unit Values
for i=1:br
    R(i,1)=(l(i,4))/Zb;
    X(i,1)=(l(i,5))/Zb;
end
for i=1:no
    P(i,1)=((m(i,2))/(1000*MVAb));
    Q(i,1)=((m(i,3))/(1000*MVAb));
end
R;
X;
P;
Q;
C=zeros(br,no);
for i=1:br
    a=l(i,2);
    b=l(i,3);
    for j=1:no
        if a==j
            C(i,j)=-1;
        end
        if b==j
            C(i,j)=1;
        end
    end
end
C;
e=1;
for i=1:no
    d=0;
    for j=1:br
        if C(j,i)==-1
            d=1;
        end
    end
    if d==0
        endnode(e,1)=i;
        e=e+1;
    end
end
endnode;
h=length(endnode);
for j=1:h
    e=2;
    
    f=endnode(j,1);
   % while (f~=1)
   for s=1:no
     if (f~=1)
       k=1;  
       for i=1:br
           if ((C(i,f)==1)&&(k==1))
                f=i;
                k=2;
           end
       end
       k=1;
       for i=1:no
           if ((C(f,i)==-1)&&(k==1));
                f=i;
                g(j,e)=i;
                e=e+1;
                k=3;
           end            
       end
     end
   end
end
for i=1:h
    g(i,1)=endnode(i,1);
end
g;
w=length(g(1,:));
for i=1:h
    j=1;
    for k=1:no 
        for t=1:w
            if g(i,t)==k
                g(i,t)=g(i,j);
                g(i,j)=k;
                j=j+1;
             end
         end
    end
end
g;
for k=1:br
    e=1;
    for i=1:h
        for j=1:w-1
            if (g(i,j)==k) 
                if g(i,j+1)~=0
                    adjb(k,e)=g(i,j+1);            
                    e=e+1;
                else
                    adjb(k,1)=0;
                end
             end
        end
    end
end
adjb;
for i=1:br-1
    for j=h:-1:1
        for k=j:-1:2
            if adjb(i,j)==adjb(i,k-1)
                adjb(i,j)=0;
            end
        end
    end
end
adjb;
x=length(adjb(:,1));
ab=length(adjb(1,:));
for i=1:x
    for j=1:ab
        if adjb(i,j)==0 && j~=ab
            if adjb(i,j+1)~=0
                adjb(i,j)=adjb(i,j+1);
                adjb(i,j+1)=0;
            end
        end
        if adjb(i,j)~=0
            adjb(i,j)=adjb(i,j)-1;
        end
    end
end
adjb;
for i=1:x-1
    for j=1:ab
        adjcb(i,j)=adjb(i+1,j);
    end
end
b=length(adjcb);

% voltage current program

for i=1:no
    vb(i,1)=0.99;
end
% for s=1:10
iterations = 1;
inertia = 1.0;
correction_factor = 2.0;
swarms = 50;

% ---- initial swarm position -----
swarm=zeros(50,7);
step = 1;
for i = 1 : 50
swarm(step, 1:7) = i;
step = step + 1;
end

swarm(:, 7) = 1000;       % Greater than maximum possible value
swarm(:, 5) = 0;          % initial velocity
swarm(:, 6) = 0 ;         % initial velocity

%% Iterations
for iter = 1 : iterations
    
    %-- position of Swarms ---
    for uu = 1 : swarms;
        swarm(uu, 1) = swarm(uu, 1) + swarm(uu, 5)/1.2;     %update u position
        swarm(uu, 2) = swarm(uu, 2) + swarm(uu, 6)/1.2;     %update v position
        u = swarm(uu, 1);
        v = swarm(uu, 2);

for i=1:no
    nlc(i,1)=conj(complex(P(i,1),Q(i,1)))/(vb(i,1));
end
nlc;
for i=1:br
    Ibr(i,1)=nlc(i+1,1);
end
Ibr;
Ibr1=Ibr;
xy=length(adjcb(1,:));
for i=br-1:-1:1
    for k=1:xy
        if adjcb(i,k)~=0
            u=adjcb(i,k);
            %Ibr(i,1)=nlc(i+1,1)+Ibr(k,1);
            Ibr(i,1)=Ibr(i,1)+Ibr(u,1);
        end
    end      
end
Ibr;
Ibr2=Ibr;

%DG1
Reall = 3715;
Reactivee = 2300;
AP_Power = sqrt((Reall )^2 + (Reactivee)^2) ;                 % Appearent Power KVA
Penetration = 30/100 ;
AP_Power = AP_Power * Penetration; 
AP_Pu = AP_Power/(1000*MVAb);   % Converting to per unit
P_Factor   = 0.85;                 % Power Factor
Real_Power = AP_Pu * P_Factor;
Rect_Power = (AP_Pu * (sin (acos (P_Factor))));
 
for ii=32:-1:2
    DG_Location=ii;
    for jj= 0:0.001:Real_Power  % Take Appearent Power Power
        DG_Size = complex(Real_Power,Rect_Power * (-1));
iiii=ii;         
    Ibr=Ibr2;
    Ibr(ii,1)=DG_Size;
    %
            if ii <= 18
    for iii= ii-1:-1:2
    jjj=iii+1;
    Ibr(iii,1)=Ibr(jjj,1)-Ibr1(jjj,1);
    end
%         if ii~=18
%          bkk=1;
%      for iii=ii+1:18
%          bss(bkk)=Ibr1(iii,1); 
%          bkk = bkk + 1 ;
%      end
%     bss=sum(bss);    
% Ibr(ii+1,1)=bss;
%     for iii=ii+1:18
%      jjj=ii+1;
%     Ibr(iii,1)=Ibr(jjj,1)-Ibr1(jjj,1);    
%     end
%         end    
            end  
%
    if ii  >= 19 && ii <= 22        
    for iii= ii-1:-1:19
    jjj=iii+1;
    Ibr(iii,1)=Ibr(jjj,1)-Ibr1(jjj,1);
    end
%         if ii~=22
%          bkk=1;
%      for iii=ii+1:19
%          bss(bkk)=Ibr1(iii,1); 
%          bkk = bkk + 1 ;
%      end
%     bss=sum(bss);   
% Ibr(ii+1,1)=bss;
%     for iii=ii+1:19
%      jjj=ii+1;
%     Ibr(iii,1)=Ibr(jjj,1)-Ibr1(jjj,1);    
%     end
%         end    
    end
%
    if ii >= 23 && ii <= 25
    for iii= ii-1:-1:23
    jjj=iii+1;
    Ibr(iii,1)=Ibr(jjj,1)-Ibr1(jjj,1);
    end
%         if ii~=25
%          bkk=1;
%      for iii=ii+1:23
%          bss(bkk)=Ibr1(iii,1); 
%          bkk = bkk + 1 ;
%      end
%     bss=sum(bss);   
% Ibr(ii+1,1)=bss;
%     for iii=ii+1:23
%      jjj=ii+1;
%     Ibr(iii,1)=Ibr(jjj,1)-Ibr1(jjj,1);    
%     end
%         end    
    end  
    %
    if ii >= 27
                            for iii= ii-1:-1:26
                            jjj=iii+1;
                            Ibr(iii,1)=Ibr(jjj,1)-Ibr1(jjj,1);
                            end
%         bkk=1;
%         if ii~=32
%                 for iii=ii+1:32
%                 bss(bkk)=Ibr1(iii,1); 
%                 bkk = bkk + 1 ;
%                 end
%             bss=sum(bss);      
%         pp=ii+1;
%         Ibr(pp,1)=bss;
%             for iii=pp+1:32
%             Ibr(iii,1)=Ibr(pp,1)-Ibr1(pp,1);    
%             end
%         end
    end
    
Ibr(1,1)=Ibr(1,1) ;
Ibr(2,1)=Ibr(1,1)-Ibr1(1,1)- DG_Size;
Ibr(3,1)=Ibr(2,1)-Ibr1(2,1)-Ibr(18,1);
Ibr(4,1)=Ibr(3,1)-Ibr1(3,1)-Ibr(22,1);
Ibr(5,1)=Ibr(4,1)-Ibr1(4,1);

%%%

for i=2:no
      g=0;
      for a=1:b 
          if xy>1
            if adjcb(a,2)==i-1 
                u=adjcb(a,1);
                vb(i,1)=((vb(u,1))-((Ibr(i-1,1))*(complex((R(i-1,1)),X(i-1,1)))));
                g=1;
            end
            if adjcb(a,3)==i-1 
                u=adjcb(a,1);
                vb(i,1)=((vb(u,1))-((Ibr(i-1,1))*(complex((R(i-1,1)),X(i-1,1)))));
                g=1;
            end
          end
        end
        if g==0
            vb(i,1)=((vb(i-1,1))-((Ibr(i-1,1))*(complex((R(i-1,1)),X(i-1,1)))));
        end
end
% s=s+1;
% end
nlc;
Ibr;
vb;
 vbp=[abs(vb)]; % angle(vb)*(180/pi)];


for i=1:no
%     va(i,2:3)=vbp(i,1:2);
      va(i,2)=vbp(i,1);
end
for i=1:no
    va(i,1)=i;
end
va;


Ibrp=abs(Ibr); % angle(Ibr)*180/pi];
PL(1,1)=0;
QL(1,1)=0;

% losses
for f=1:br
    Pl(f,1)=(Ibrp(f,1)^2)*R(f,1);
    Ql(f,1)=X(f,1)*(Ibrp(f,1)^2);
    PL(1,1)=PL(1,1)+Pl(f,1);
    QL(1,1)=QL(1,1)+Ql(f,1);
end

Plosskw=(Pl)*100000;
Qlosskw=(Ql)*100000;
PL=(PL)*100000;
QL=(QL)*100000;


voltage = vbp(:,1);
% angle = vbp(:,2)*(pi/180);


PL_Sum=PL+QL;
if PL_Cmp > PL_Sum
   PL_Cmp = PL_Sum; 
   DG_Sizr=DG_Size;
   DG_Loc=DG_Location;
   voltr=voltage;
%    angle;
   PLr=PL;
   QLr=QL;
   Plosskwr=Plosskw;
   Qlosskwr=Qlosskw;
   Ibrr=Ibr;
end
    [temp, gbest] = min(swarm(:, 7));        % gbest position
    
    %--- updating velocity vectors
    for vv = 1 : swarms
        swarm(vv, 5) = rand*inertia*swarm(vv, 5) + correction_factor*rand*(swarm(vv, 3)...
            - swarm(vv, 1)) + correction_factor*rand*(swarm(gbest, 3) - swarm(vv, 1));   % u velocity parameters
        swarm(vv, 6) = rand*inertia*swarm(vv, 6) + correction_factor*rand*(swarm(vv, 4)...
            - swarm(vv, 2)) + correction_factor*rand*(swarm(gbest, 4) - swarm(vv, 2));   % v velocity parameters
    end
    
    % Plotting the swarm
%     clf    
%     plot(swarm(:, 1), swarm(:, 2), 'x');   % drawing swarm movements
%     axis([-2 50 -2 50]);
% pause(.0000000000000000001)      
    end
end
    end
end

toc;


%    DG_Sizr;
   DG_Sizrr=(abs(DG_Sizr))*1000*100;

%
Plosskwr(33,1)=PLr;
Qlosskwr(33,1)=QLr;

   DG_Location = DG_Loc + 1;
   
sprintf('Power-Loss= %d KW, Power-Loss= %d KVAr' ,PLr,QLr')  
sprintf('DG Location= %d , DG Power = %d KVA' ,DG_Location,DG_Sizrr') 
Sr=(1:33)';

plot(m(:,1),abs(voltr));

    %% EXCEL FOR DG
% %EXCEL
T =table(Sr,Plosskwr,Qlosskwr,voltr);        
T(:,1:4);
excel_file = 'DG1_IEEE33.xlsx';
writetable(T,excel_file,'Sheet',1,'Range','H1');

% Email: mraza.engg@gmail.com
