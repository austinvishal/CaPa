function tension_space_Hrep= tension_space_polytope(t_min,t_max,n)
%t_min: minimum allowable tension value
%t_max: maximum allowable tension value
%tension_space_Vrep : tension space polytope in vertices representation
A_desired= vertcat(eye(n,n),-1*eye(n,n))
B_desired= vertcat(t_max*ones([n,1]),-t_min*ones([n,1]))
tension_space_Hrep=Polyhedron(A_desired, B_desired); % call from MPT3 toolbox eth
end

