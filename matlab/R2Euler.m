function [euler] = R2Euler(R)

euler = zeros(3,1);

euler(2) = -asin(R(3,1));

euler(1) = atan2(R(3,2) / cos(euler(1)), R(3,3) / cos(euler(1)));

euler(3) = atan2(R(2,1) / cos(euler(1)), R(1,1) / cos(euler(1)));