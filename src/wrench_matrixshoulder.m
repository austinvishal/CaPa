function W= wrench_matrixshoulder(wld_position_base_pts_, wld_pose_tool_, tool_position_plat_pts_)
%input: normalized_cable_vectors
%output : wrench matrix

%% wld_position_base_pts 6x3, tool_position_plat_pts 6x3
p=wld_pose_tool_(1:3,4) %3x1
W=zeros(3,6); %3 dof 6 cables nxm %3x6
for i=1:size(wld_position_base_pts_,1)
   world_position_plat_pts_(:,i) =wld_pose_tool_(1:3,1:3)*tool_position_plat_pts_(i,:)'
   cable_vec_(:,i)=(wld_position_base_pts_(i,:)'-p-world_position_plat_pts_(:,i));
   cable_vec_normalized(:,i) = cable_vec_(:,i) / norm(cable_vec_(:,i));
%   W(1:3,i)=cable_vec_normalized(:,i);
%   W(4:6,i)=cross(world_position_plat_pts_(:,i), cable_vec_normalized(:,i));
    W(1:3,i)=cross(world_position_plat_pts_(:,i), cable_vec_normalized(:,i));
end

end