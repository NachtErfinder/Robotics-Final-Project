function Theta = FindTheta(R,Z_target,V)
R = R/1000;
Z_target = Z_target/1000;
a = -9.81;
Theta = -atand((V^2-sqrt(V^4-a*(a*R^2+2*-Z_target*V^2)))/(a*R));
end