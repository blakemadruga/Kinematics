%Forward and inverse kinematics of a crane

%q = joint parameters: change ONLY these to change the system, 4 variables
% = 4 DOF
theta1 = -pi/4; 
zdist2 = 5; % 1 <= zdist2 <= 10
zdist3 = 20; % 0 <= zdist3 <= 20
theta4 = 0;

q = [theta1,zdist2,zdist3,theta4];
index = [0];
for ele = index

if(ele==0)
   q  = [0,1,0,0];
elseif(ele==1)
   q  = [theta1,1,0,0];
elseif(ele==2)
   q  = [theta1,zdist2,0,0];
elseif(ele==3)
   q  = [theta1,zdist2,zdist3,0];
elseif(ele==4)
   q  = [theta1,zdist2,zdist3,theta4];
end
   
%Denavit-Hartenberg Parameters:
%theta = angle about previous z, from old x to new x
%alpha = angle about common normal, from old  z axis to new z axis
%r     = length of the common normal
%d     = offset along previous z to the common normal

%     theta           alpha r d 
DH = [(q(1) - pi/2) -pi/2 0 0;        %Frame 1 to frame 0
      0               -pi/2 0 q(2);   %Frame 2 to frame 1
      0               0     0 q(3);   %Frame 3 to frame 2
      q(4)            0     0 1;];      %Frame 4 to frame 3
      
T01 = Transform(DH(1,:));       %T1 from T0
T12 = Transform(DH(2,:));       %T2 from T1
T23 = Transform(DH(3,:));       %T3 from T2
T34 = Transform(DH(4,:));       %T4 from T3

T04 = T01*T12*T23*T34           %T4 in base frame transform

unitv = eye(4); %define the unit vectors on each axis 

P01 = T01*unitv; %calculate the position frame 1 from the base frame 
P01origin = ([P01(1:3,4),P01(1:3,4),P01(1:3,4)]).'; %specify the origin from the P01 matrix
P01T = (P01(1:3,1:3)).'; %transpose the matrix for display purposes

P02 = T01*T12*unitv;
P02origin = ([P02(1:3,4),P02(1:3,4),P02(1:3,4)]).';
P02T = (P02(1:3,1:3)).';
l2 = P02(1:3,4).';
l2origin = P01(1:3,4).';

P03 = T01*T12*T23*unitv;
P03origin = ([P03(1:3,4),P03(1:3,4),P03(1:3,4)]).';
P03T = (P03(1:3,1:3)).';
l3 = (P03(1:3,4).')-l2;
l3origin = P02(1:3,4).';

P04 = T01*T12*T23*T34*unitv;
P04origin = ([P04(1:3,4),P04(1:3,4),P04(1:3,4)]).';
P04T = (P04(1:3,1:3)).';
l4 = (P04(1:3,4).')-l3-l2;
l4origin = P03(1:3,4).'; 


P0origin = zeros(3);
P0T = eye(3);
    
unitvrays = [
  P0origin  P0T;
  P01origin P01T;
  P02origin P02T;
  P03origin P03T;
  P04origin P04T;
];

linkrays = [
    l2origin l2
    l3origin l3
    l4origin l4
    ];

if(ele==0)
    endEffOri0 = P04origin;
    endEffVect0 = P04origin - P04origin;
    trajrays = [
        endEffOri0 endEffVect0
        ]
elseif(ele==1)
    endEffOri1 = endEffOri0 + endEffVect0
    endEffVect1 = P04origin - endEffOri0;
    trajrays = [
        endEffOri1 endEffVect1
        ];
elseif(ele==2)
    endEffOri2 = endEffOri1 + endEffVect1
    endEffVect2 = P04origin - endEffOri1 - endEffVect1;
    trajrays = [
        endEffOri2 endEffVect2
        ];
elseif(ele==3)
    endEffOri3 = endEffOri2 + endEffVect2;
    endEffVect3 = P04origin - endEffOri2 - endEffVect2;
    trajrays = [
        endEffOri3 endEffVect3
        ];
elseif(ele==4)
    endEffOri4 = endEffOri3 + endEffVect3;
    endEffVect4 = P04origin - endEffOri3 - endEffVect3;
    trajrays = [
        endEffOri4 endEffVect4
        ];
end
%trajrays = 

% quiver plot
quiver3( unitvrays( :,1 ), unitvrays( :,2 ), unitvrays( :,3 ), unitvrays( :,4 ), unitvrays( :,5 ), unitvrays( :,6 ),0,'Color',[.6 0 0]);
hold on;
quiver3( linkrays( :,1 ), linkrays( :,2 ), linkrays( :,3 ), linkrays( :,4 ), linkrays( :,5 ), linkrays( :,6 ),0,'Color',[0 .6 0]);
quiver3( trajrays( :,1 ), trajrays( :,2 ), trajrays( :,3 ), trajrays( :,4 ), trajrays( :,5 ), trajrays( :,6 ),0,'Color',[0 0 .6]);

title('Kinematic representation of the crane');
axis([-11 11 -11 11 -22 1]);
end
%----------------------
%-----Inverse Kinematics-----%
% Robot's mission is to position the jaws of a gripper (end effector)
% to an object of interest, involving the following tasks:

q1rad = atan2(T04(2,4),T04(1,4)); %rotation of the main boom to an object's position
q1 = rad2deg(q1rad) 
q2 = sqrt((T04(2,4)^2)+T04(1,4)^2) %Position the linear trolley to object

q3sol1rad = -(asin(T04(1,1))-q1rad); % Rotation of end effector to optimal position for object
q3sol1 = rad2deg(q3sol1rad);
checkmat1 = Check(T04(1:2,1:2),(q1rad-q3sol1rad));

q3sol2rad = -(-pi-asin(T04(1,1))-q1rad);
q3sol2 = rad2deg(-(-pi-asin(T04(1,1))-q1rad));
checkmat2 = Check(T04(1:2,1:2),(q1rad-q3sol2rad));

if (checkmat1 == 1)
    q3 = q3sol1
elseif(checkmat2 == 1)
    if (q3sol2 >180)
        q3 = 360 - q3sol2
    else
        q3 = q3sol2
    end
    
end

q4 = -T04(3,4) - DH(4,4) %Lower jaws of gripper into place

rad2deg(theta1-theta4);


function check = Check(A,theta)
check = 0;
B = [sin(theta)  -cos(theta);
     -cos(theta) -sin(theta)];
if (A(1,1) - B(1,1) < 0.01)
    if(A(1,2) - B(1,2) < 0.01)
        if (A(2,1) - B(2,1) < 0.01)
            if (A(2,1) - B(2,1) < 0.01)
                 check = 1;
            end
        end
    end
end

end

function Tn = Transform(DH)
Tn =  [cos(DH(1,1)) -sin(DH(1,1))*cos(DH(1,2)) sin(DH(1,1))*sin(DH(1,2))  DH(1,3)*cos(DH(1,1));
       sin(DH(1,1)) cos(DH(1,1))*cos(DH(1,2))  -cos(DH(1,1))*sin(DH(1,2)) DH(1,3)*sin(DH(1,1));
       0          sin(DH(1,2))             cos(DH(1,2))             DH(1,4);
       0          0              0              1;];

end