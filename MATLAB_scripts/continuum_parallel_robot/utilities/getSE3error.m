function Psi = getSE3error(Q, r, Q_ref, r_ref)

R_ref = Q2R(Q_ref);
R_X1 = Q2R(Q);

R = R_ref'*R_X1;
EUL = rotm2eul(R);
yaw   = EUL(1);
pitch = EUL(2);
roll  = EUL(3);


error_p = r - r_ref;

Psi = [roll
       pitch
       yaw
       error_p];

end