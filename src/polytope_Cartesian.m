function polycart=polytope_Cartesian(W,tension_space_Vrep,scale)
% W: wrench matrix 
% tensionsapceVrep: tension space polytope vertices 
% outpute: wrench space vertices
% this function lets you project from tension space/configuration space to wrench space ie cartesian space using
% wrench matrix
% polyhedron.V gives the vertices using MPT3 toolbox

V_main=zeros(length(tension_space_Vrep.V),size(W,1))
length(tension_space_Vrep.V)
for i=1:length(tension_space_Vrep.V)
V_main(i,:)=(W*( tension_space_Vrep.V(i,:))')'
end
polycart=scale*Polyhedron(V_main);
end