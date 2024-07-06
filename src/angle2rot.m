function R=angle2rot(alpha,beta,gamma)
R=Rz(alpha)*Ry(beta)*Rz(gamma);
end