function [check_vel1,check_vel2,position_rank,value,iteration,max_FES,result]= CHCLPSO_new_Drone(data,num_particle,range_max,range_min,dimension,max_iteration,max_FES) 

check_vel=[];
check_vel1=[];
check_vel2=[];

num_g=num_particle;
num_g1=10;
num_g2=num_g-num_g1;

j=0:(1/(num_g-1)):1; % Learning Probability Curve Pc
j=j*10;
Pc=ones(dimension,1)*(0.0+((0.25).*(exp(j)-exp(j(1)))./(exp(j(num_g))-exp(j(1))))); 

Weight =0.99-(1:max_iteration)*0.79/max_iteration; % Inertia Weight

K=3-(1:max_iteration)*1.5/max_iteration;% Acceleration Coefficients
c1=2.5-(1:max_iteration)*2/max_iteration;
c2=0.5+(1:max_iteration)*2/max_iteration;
c3=0.5+(1:max_iteration)*2/max_iteration;

% Initialization
%  range_min=range*ones(num_g,1); % Range for initial swarm's elements
%  range_max=range(2)*ones(num_g,1);
interval = repmat(range_max - range_min,num_g,1);
v_max=interval*0.2; 
v_min=-v_max;
% switch method
%     case 2
%         pos = Partcl_Init(data,C,num_g);
%     case 1
       pos = range_min+ interval.*rand(num_g,dimension);    %  position
% end

vel =v_min+(v_max-v_min).*rand(num_g,dimension); % velocity


k=1; 
fitcount=0;
result_data=zeros(1,max_FES);
result=zeros(1,num_g); 

for i=1:num_g
     out1=eval_profit1(pos(i,:),data);
     result(i) = out1;
     fitcount=fitcount+1;
     result_data(fitcount)=result(i);
end

[gbest_val,g_index]=min(result);
gbest_pos=pos(g_index,:); 

pbest_pos=pos; 
pbest_val=result;

obj_func_slope=zeros(num_g,1);
fri_best=(1:num_g)'*ones(1,dimension);

        
for i=1:num_g1 % Updateding examplers for; group 1
    fri_best(i,:)=i*ones(1,dimension);
    friend1=ceil(num_g1*rand(1,dimension));
    friend2=ceil(num_g1*rand(1,dimension));
    friend=(pbest_val(friend1)<pbest_val(friend2)).*friend1+(pbest_val(friend1)>=pbest_val(friend2)).*friend2;
    toss=ceil(rand(1,dimension)-Pc(:,i)');
    if toss==ones(1,dimension)
       temp_index=randperm(dimension);
       toss(1,temp_index(1))=0;
       clear temp_index;
    end
    fri_best(i,:)=(1-toss).*friend+toss.*fri_best(i,:);
    fco = fri_best(i,:) == 0;
    fcos = sum(fco);
    if fcos>0
        ss = ceil(num_g*rand(fcos,1));
        co = 0;
        for iii = 1:dimension
            if fco(iii) == 1
                co = co + 1;
                fri_best(i,iii) = ss(co);
            end
        end
    end
            
    for d=1:dimension
        fri_best_pos(i,d)=pbest_pos(fri_best(i,d),d);
    end
end

for i=num_g1+1:num_g % Updateding examplers for; group 1
    fri_best(i,:)=i*ones(1,dimension);
    friend1=ceil(num_g*rand(1,dimension));
    friend2=ceil(num_g*rand(1,dimension));
    friend=(pbest_val(friend1)<pbest_val(friend2)).*friend1+(pbest_val(friend1)>=pbest_val(friend2)).*friend2;
    toss=ceil(rand(1,dimension)-Pc(:,i)');
    if toss==ones(1,dimension)
       temp_index=randperm(dimension);
       toss(1,temp_index(1))=0;
       clear temp_index;
    end
    fri_best(i,:)=(1-toss).*friend+toss.*fri_best(i,:);
    fco = fri_best(i,:) == 0;
    fcos = sum(fco);
    if fcos>0
        ss = ceil(num_g*rand(fcos,1));
        co = 0;
        for iii = 1:dimension
            if fco(iii) == 1
                co = co + 1;
                fri_best(i,iii) = ss(co);
            end
        end
    end
    for d=1:dimension
        fri_best_pos(i,d)=pbest_pos(fri_best(i,d),d);
    end
end


while k<=max_iteration && fitcount<=max_FES      
        % group 1
          
        delta_g1=(K(k).*rand(num_g1,dimension).*(fri_best_pos(1:num_g1,:)-pos(1:num_g1,:)));
        vel_g1=Weight(k)*vel(1:num_g1,:)+delta_g1;
        vel_g1=((vel_g1<v_min(1:num_g1,:)).*v_min(1:num_g1,:))+((vel_g1>v_max(1:num_g1,:)).*v_max(1:num_g1,:))+(((vel_g1<v_max(1:num_g1,:))&(vel_g1>v_min(1:num_g1,:))).*vel_g1);
        pos_g1=pos(1:num_g1,:)+vel_g1;
        
%         switch family
%             case 1        % group 2
                gbest_pos_temp=repmat(gbest_pos,num_g2,1);
                delta_g2=(c1(k).*rand(num_g2,dimension).*(fri_best_pos(num_g1+1:end,:)-pos(num_g1+1:end,:)))+(c2(k).*rand(num_g2,dimension).*(gbest_pos_temp-pos(num_g1+1:end,:)));
                vel_g2=Weight(k)*vel(num_g1+1:end,:)+delta_g2;
                vel_g2=((vel_g2<v_min(num_g1+1:end,:)).*v_min(num_g1+1:end,:))+((vel_g2>v_max(num_g1+1:end,:)).*v_max(num_g1+1:end,:))+(((vel_g2<v_max(num_g1+1:end,:))&(vel_g2>v_min(num_g1+1:end,:))).*vel_g2);
                pos_g2=pos(num_g1+1:end,:)+vel_g2;
%             case 2 
%                 out =  eval_profit1(pos,data);
%                 fb  = Family_best_Part(out.index,C,pos,pbest_val);
%                 fbt = zeros(num_g2,dimension);
%                 for i = num_g1+1:num_g
%                     fbt1 = sum(fb-repmat(pos(i,:),C,1));
%                     fbl = fbt1>0;
%                     if sum(fbl)>C-1
%                         fbt(i-num_g1,:) = sum(fb-repmat(pos(i,:),C,1))./C;
%                     else
%                         fbt(i-num_g1,:) = sum(fb-repmat(pos(i,:),C,1))./sum(fbl);
%                     end
%                 end                
%                 gbest_pos_temp=repmat(gbest_pos,num_g2,1);
%                 delta_g2=(c1(k).*rand(num_g2,dimension).*(fri_best_pos(num_g1+1:end,:)-pos(num_g1+1:end,:)))+(c2(k).*rand(num_g2,dimension).*(gbest_pos_temp-pos(num_g1+1:end,:)))...
%                     +(c3(k).*rand(num_g2,dimension).*(fbt));
%                 vel_g2=Weight(k)*vel(num_g1+1:end,:)+delta_g2;
%                 vel_g2=((vel_g2<v_min(num_g1+1:end,:)).*v_min(num_g1+1:end,:))+((vel_g2>v_max(num_g1+1:end,:)).*v_max(num_g1+1:end,:))+(((vel_g2<v_max(num_g1+1:end,:))&(vel_g2>v_min(num_g1+1:end,:))).*vel_g2);
%                 pos_g2=pos(num_g1+1:end,:)+vel_g2;
%         end

         % whole group
    
        pos=[pos_g1;pos_g2];
        vel=[vel_g1;vel_g2];
   
        % Evaluate fitness
         for i=1:num_g   
            if (sum(pos(i,:)>range_max)+sum(pos(i,:)<range_min)==0)
                out1=eval_profit1(pos(i,:),data); 
                result(i) = out1;
                result_data(fitcount)=result(i);
                fitcount=fitcount+1;
                if fitcount>=max_FES
                   break;
                end
            end
            if  result(i)<pbest_val(i) % update pbest value and position
                pbest_pos(i,:)=pos(i,:);   
                pbest_val(i)=result(i);
                obj_func_slope(i)=0;
            else
                obj_func_slope(i)=obj_func_slope(i)+1;
            end
            
            if  pbest_val(i)<gbest_val % update gbest value and postion
                gbest_pos=pbest_pos(i,:); 
                gbest_val=pbest_val(i);
            end   
         end 
    
        for i=1:num_g1 % updateding exampler for group 1
            if obj_func_slope(i)>5
                fri_best(i,:)=i*ones(1,dimension); % for its own pbest
                friend1=ceil(num_g1*rand(1,dimension));
                friend2=ceil(num_g1*rand(1,dimension));
                friend=(pbest_val(friend1)<pbest_val(friend2)).*friend1+(pbest_val(friend1)>=pbest_val(friend2)).*friend2;
                toss=ceil(rand(1,dimension)-Pc(:,i)');
            
                if toss==ones(1,dimension)
                    temp_index=randperm(dimension);
                    toss(1,temp_index(1))=0;
                    clear temp_index;
                end
            
                fri_best(i,:)=(1-toss).*friend+toss.*fri_best(i,:);
                fco = fri_best(i,:) == 0;
                fcos = sum(fco);
                if fcos>0
                    ss = ceil(num_g*rand(fcos,1));
                    co = 0;
                    for iii = 1:dimension
                        if fco(iii) == 1
                            co = co + 1;
                            fri_best(i,iii) = ss(co);
                        end
                    end
                end
                for d=1:dimension
                    fri_best_pos(i,d)=pbest_pos(fri_best(i,d),d);
                end
                obj_func_slope(i)=0;
            end
        end % updating exampler for group 1
    
        for i=num_g1+1:num_g % updating exampler for group 2
            if obj_func_slope(i)>5
                fri_best(i,:)=i*ones(1,dimension);
                friend1=ceil(num_g*rand(1,dimension));
                friend2=ceil(num_g*rand(1,dimension));
                friend=(pbest_val(friend1)<pbest_val(friend2)).*friend1+(pbest_val(friend1)>=pbest_val(friend2)).*friend2;
                toss=ceil(rand(1,dimension)-Pc(:,i)');
            
                if toss==ones(1,dimension)
                    temp_index=randperm(dimension);
                    toss(1,temp_index(1))=0;
                    clear temp_index;
                end
            
                fri_best(i,:)=(1-toss).*friend+toss.*fri_best(i,:);
                fco = fri_best(i,:) == 0;
                fcos = sum(fco);
                if fcos>0
                    ss = ceil(num_g*rand(fcos,1));
                    co = 0;
                    for iii = 1:dimension
                        if fco(iii) == 1
                            co = co + 1;
                            fri_best(i,iii) = ss(co);
                        end
                    end
                end
                for d=1:dimension
                fri_best_pos(i,d)=pbest_pos(fri_best(i,d),d);
                end
                obj_func_slope(i)=0;
            end
        end % updating exampler for group 2
        
         check_vel=[check_vel (sum(abs(vel'))/dimension)'];
         check_vel1=[check_vel1 sum(check_vel(1:num_g1,end))/num_g1];
         check_vel2=[check_vel2 sum(check_vel(num_g1+1:end,end))/num_g2];
        
         
        if fitcount>=max_FES
            break;
        end 
        
%         if (k==max_iteration)&&(fitcount<max_FES)
%             k=k-1;
%         end
%         
  k=k+1;
end

% fprintf('Output\n');
check_vel1=check_vel1./(range(2)-range(1));
check_vel2=check_vel2./(range(2)-range(1));
position=gbest_pos;
value=gbest_val;
iteration=k;
max_FES=fitcount;
result=result_data;
% Error=value-f_bias(func_num);
[~,position_rank] = sort(position,2,"descend");
 end
