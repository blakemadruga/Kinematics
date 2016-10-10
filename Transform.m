function Tn = Transform(DH)
    Tn =  [cos(DH(1,1)) -sin(DH(1,1))*cos(DH(1,2)) sin(DH(1,1))*sin(DH(1,2))  DH(1,3)*cos(DH(1,1));
       sin(DH(1,1)) cos(DH(1,1))*cos(DH(1,2))  -cos(DH(1,1))*sin(DH(1,2)) DH(1,3)*sin(DH(1,1));
       0          sin(DH(1,2))             cos(DH(1,2))             DH(1,4);
       0          0              0              1;];

end