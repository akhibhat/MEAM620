function [path_2] = addingpoints(path_2)
min_dist2=1;
min_dist=1.2;
diff_path=abs(diff(path_2));
diff_path_true=diff(path_2);
diff_1=vecnorm(diff_path,2,2);
locs=find(diff_1>min_dist2);
nu_long_paths=size(locs,1);
nu_inter=zeros(nu_long_paths,1);
for j=1:nu_long_paths
    nu_inter(j)=floor(max(diff_path(locs(j),:))/min_dist);
end
locas=find(nu_inter==0)
locs(locas)=[];
nu_inter(locas)=[];
nu_long_paths=nu_long_paths-size(locas,1);
stack=zeros(sum(nu_inter),3);
i=1;
for j=1:nu_long_paths
   
        increment=diff_path_true(locs(j),:)/(nu_inter(j)+0.1);
        
        for k=1:nu_inter(j)
            stack(i,:)=path_2(locs(j),:)+k*increment;
            i=i+1;
        end
    
end
i=1;
nu_inter=[0;nu_inter]
for k=1:nu_long_paths
    path_2=[path_2(1:locs(k)+sum(nu_inter(1:k)),:);stack(sum(nu_inter(1:k))+1:sum(nu_inter(1:k))+nu_inter(k+1),:);path_2(locs(k)+1+sum(nu_inter(1:k)):end,:)]
    i=i+1
end
