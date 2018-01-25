clc,clear;
%% Forward Kinematics
d1 = sym('d1');
d2 = sym('d2');
d3 = sym('d3');
d5 = sym('d5');
t2 = sym('theta2');
t3 = sym('theta3');
t4 = sym('theta4');
zeroTone=[1 0 0 0;0 1 0 0;0 0 1 d1;0 0 0 1];
oneTtwo=[cos(t2) -sin(t2) 0 0;sin(t2) cos(t2) 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 0;0 0 1 d2;0 0 0 1];
twoTthree=[1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1]*[cos(t3) -sin(t3) 0 0;sin(t3) cos(t3) 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 0;0 0 1 d3;0 0 0 1];
threeTfour=[1 0 0 0;0 0 1 0;0 -1 0 0;0 0 0 1]*[cos(t4) -sin(t4) 0 0;sin(t4) cos(t4) 0 0;0 0 1 0;0 0 0 1];
fourTfive=[-1 0 0 0;0 -1 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 0;0 0 1 d5;0 0 0 1];
zeroTfive=zeroTone*oneTtwo*twoTthree*threeTfour*fourTfive
p5=[0;0;0;1];
p0=zeroTfive*p5
%% Inverse Kinematics
% known values
d2 = 15;
d3 = 5;
d5 = 10;
% symbolic variable for J
 q1 = sym('q1');
 q2 = sym('q2');
 q3 = sym('q3');
 q4 = sym('q4');
% find the jacobian expression
J=jacobian([-cos(q2)*sin(q3)*d5+sin(q2)*d3,-sin(q2)*sin(q3)*d5-cos(q2)*d3,cos(q3)*d5+d2+q1,1],[q1,q2,q3,q4])
x_0=[-1.46;-8.54;37.1;1];
% guessed values
d1 = 0;
t2 = 0;
t3 = 0;
t4 = 1;
q=[d1;t2;t3;t4];
zeroTone=[1 0 0 0;0 1 0 0;0 0 1 q(1);0 0 0 1];
oneTtwo=[cos(q(2)) -sin(q(2)) 0 0;sin(q(2)) cos(t2) 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 0;0 0 1 d2;0 0 0 1];
twoTthree=[1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1]*[cos(q(3)) -sin(q(3)) 0 0;sin(q(3)) cos(q(3)) 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 0;0 0 1 d3;0 0 0 1];
threeTfour=[1 0 0 0;0 0 1 0;0 -1 0 0;0 0 0 1]*[cos(q(4)) -sin(q(3)) 0 0;sin(q(4)) cos(q(4)) 0 0;0 0 1 0;0 0 0 1];
fourTfive=[-1 0 0 0;0 -1 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 0;0 0 1 d5;0 0 0 1];
zeroTfive=zeroTone*oneTtwo*twoTthree*threeTfour*fourTfive;
x_d=zeroTfive*p5;
while(sum(abs((x_d-x_0))<0.05)<4) % check if the guess q yields a good x
    dx=(x_d-x_0)*0.001;
    J=[0 10*sin(q(3))*sin(q(2))+5*cos(q(2)) -10*cos(q(3))*cos(q(2)) 0;0 -10*sin(q(3))*cos(q(2))+5*sin(q(2)) -10*cos(q(3))*sin(q(2)) 0;1 0 -10*sin(q(3)) 0;0 0 0 0];
    J_inv=pinv(J); % the pseudo inverse of J
    dq=J_inv*dx;
    q=q-dq;
    zeroTone=[1 0 0 0;0 1 0 0;0 0 1 q(1);0 0 0 1];
    oneTtwo=[cos(q(2)) -sin(q(2)) 0 0;sin(q(2)) cos(q(2)) 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 0;0 0 1 d2;0 0 0 1];
    twoTthree=[1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1]*[cos(q(3)) -sin(q(3)) 0 0;sin(q(3)) cos(q(3)) 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 0;0 0 1 d3;0 0 0 1];
    threeTfour=[1 0 0 0;0 0 1 0;0 -1 0 0;0 0 0 1]*[cos(q(4)) -sin(q(4)) 0 0;sin(q(4)) cos(q(4)) 0 0;0 0 1 0;0 0 0 1];
    fourTfive=[-1 0 0 0;0 -1 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 0;0 0 1 d5;0 0 0 1];
    zeroTfive=zeroTone*oneTtwo*twoTthree*threeTfour*fourTfive;
    x_d=zeroTfive*p5;
end
% Convert the angles from radians to degrees
q(2)=q(2)*180/pi;
q(3)=q(3)*180/pi;
q(4)=q(4)*180/pi;
q
x_d
