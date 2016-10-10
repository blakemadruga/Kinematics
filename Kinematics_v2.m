%Forward and inverse kinematics of a crane

%q = joint parameters: change ONLY these to change the system, 4 variables.
%Influences the position of the end effector. 
% = 4 DOF
% Object of interest position
theta1 = -pi/2; % Link one rotary
zdist2 = 10; %Link two is prismatic
zdist3 = 25; %Link three is prismatic
theta4 = pi/12; %Link four is rotary

%Physical Geometric configuration of the Crane, defining coordinate systems
%-----------------------------------------

%Link 1: Rotary joint at base of the crane
t1 = theta1 -pi/2;     %angle about previous z, from old x to new x
a1 = -pi/2;     %angle about common normal, from old  z axis to new z axis
r1 = 0;         %length of the common normal
d1 = 0;         %offset along previous z to the common normal

%Link 2: Prismatic joint at the horizontal trolley (Positioning)
t2 = 0;
a2 = -pi/2;
r2 = 0;
d2 = zdist2;

%Link 3: Prismatic joint for the lowering of the end effector (and lifting)
t3 = 0;
a3 = 0;
r3 = 0;
d3 = zdist3;

%Link 4: Rotary joint for the positioning of the object of interest
%Note: d4 is some thickness of the physical clamping jaws, from the point
t4 = theta4;
a4 = 0;
r4 = 0;
d4 = 1;

t = t1;
a = a1;
r = r1;
d = d1;

%Defining the DH Parameter set for transformation by the Transform function
DH0 = [t1 a1 r1 d1;
      t2 a2 r2 d2;
      t3 a3 r3 d3;
      t4 a4 r4 d4;]; 
  
%Transformation matrix to shift RF(i) to RF(i-1)
%Note, using standard DH, not the modified DH parameters
Tn = [cos(t)  -sin(t)*cos(a) sin(t)*sin(a)  r*cos(t);
       sin(t)  cos(t)*cos(a) -cos(t)*sin(a) r*sin(t);
       0       sin(a)         cos(a)            d;
       0          0              0              1;];
      
Tn1 = Transform(DH0(1,:));       %Transforming T1 RF from T0
Tn2 = Transform(DH0(2,:));       %Transforming T2 RF from T1
Tn3 = Transform(DH0(3,:));       %Transforming T3 RF from T2
Tn4 = Transform(DH0(4,:));       %Transforming T4 RF from T3

P1 = eye(4); %4x4 Identity matrix
P2 = eye(4); %4x4 Identity matrix
P3 = eye(4); %4x4 Identity matrix
P4 = eye(4); %4x4 Identity matrix
   
%Position of link 1
P01 = Tn1*P1 
P01dist = P01(1:3,4)
P01T = (P01(1:3,1:3)).';

P02 = Tn1*Tn2*P2; 
P02dist = [P02(1:3,4),P02(1:3,4),P02(1:3,4)];
P02origin = P02dist.';
P02T = (P02(1:3,1:3)).';

P03 = Tn1*Tn2*Tn3*P3;
P03dist = [P03(1:3,4),P03(1:3,4),P03(1:3,4)];
P03origin = P03dist.'
P03T = (P03(1:3,1:3)).';

P04 = Tn1*Tn2*Tn3*Tn4*P4;
P04dist = [P04(1:3,4),P04(1:3,4),P04(1:3,4)]
P04origin = P04dist.'
P04T = (P04(1:3,1:3)).';

Tcom = Tn1*Tn2*Tn3*Tn4

P0origin = zeros(3);
P0T = eye(3);
    
rays = [
  P0origin  P0T;
  P0origin  P01T;
  P02origin P02T;
  P03origin P03T;
  P04origin P04T;
  %o2 l3;
] ;

% quiver plot
quiver3( rays( :,1 ), rays( :,2 ), rays( :,3 ), rays( :,4 ), rays( :,5 ), rays( :,6 ),0,'Color',[.6 0 0]);

%----------------------
%-----Inverse Kinematics-----%
% Robot's mission is to position the jaws of a gripper (end effector)
% to an object of interest, involving the following tasks:

q1rad = atan2(Tcom(2,4),Tcom(1,4)); %rotation of the main boom to an object's position
q1 = rad2deg(q1rad) 
q2 = sqrt((Tcom(2,4)^2)+Tcom(1,4)^2) %Position the linear trolley to object
q3rad = -(asin(Tcom(1,1))-q1rad); % Rotation of end effector to optimal position for object
q3 = rad2deg(q3rad)
q4 = -zdist3 - d4 %Lower jaws of gripper into place 



