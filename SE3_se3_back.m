function [ se3] = SE3_se3_back( SE3 )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    R=SE3(1:3,1:3);
    theta=acos((trace(R)-1)/2);
    lnR=(theta/(2*sin(theta)))*(R-R');
    w=[-lnR(2,3) lnR(1,3) -lnR(1,2)];
    wx=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
    if(theta==0)
        Vin=eye(3);
    else
%         A=sin(theta)/theta;
%         B=(1-cos(theta))/(theta^2);
%         Vin=eye(3)-(1/2)*wx+(1/(theta^2))*(1-(A/(2*B)))*(wx*wx);
    A = (1 + cos(theta))/sin(theta);
    Vin=eye(3)-(1/2)*wx+(1/(theta^2))*(1-(1/2 * theta * A))*(wx*wx);        
    end
    u=Vin*SE3(1:3,4);
    se3=[u' w];

end