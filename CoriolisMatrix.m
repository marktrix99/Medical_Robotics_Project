function Cq=CoriolisMatrix(q,dq)
global m1 m2  lc2 l1 l2 
q1=q(1);
q2=q(2);
dq1=dq(1);
dq2=dq(2);
Cq=[dq1*l1*lc2*m2*sin(q2) - (3*dq2*l1*lc2*m2*sin(q2))/2, (dq1*l1*lc2*m2*sin(q2))/2 - dq2*l1*lc2*m2*sin(q2);
 dq1*l1*lc2*m2*sin(q2) - (dq2*l1*lc2*m2*sin(q2))/2,                         (dq1*l1*lc2*m2*sin(q2))/2];
end