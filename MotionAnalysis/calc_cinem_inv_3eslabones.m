function Q = calc_cinem_inv_3eslabones(x,y,l1,l2,l3,lado)

    D = (x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2);
    
    theta2 = atan2(lado*(sqrt(1-D^2)), D);
    
    theta1 = atan2(y,x) - atan2(l2*sin(theta2), l1 + l2*cos(theta2));
    
    P = l1*sin(theta1);
    x1 = l1*cos(theta1);
    M = y - P;
      
    E = (x1^2 + (M + l3)^2 - l3^2 - l2^2)/(2*l3*l2);
    theta3 = atan2(lado*(sqrt(1-E^2)), E);
    
    Q = [theta1, theta2, theta3];

end