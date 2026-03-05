% calcolo lo Jacobiano dell'end effector
function J = Jacobian_ee_transpose (q1, q2, l1, l2)
        J = [-l1*sin(q1)-l2*sin(q1+q2), -l2*sin(q1+q2);
            l1*cos(q1)+l2*cos(q1+q2), l2*cos(q1+q2)]; 
        J_t = J
end