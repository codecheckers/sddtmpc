function [min_sin,max_sin,min_cos,max_cos] = get_border_sin_cos(theta_nominal,theta_error_set)
%{
This function caluclates maxium and minimum values of sine and cosine functios given possible values of real direction
:float theta_nominal: nominal direction
:vector theta_error_set: minimum and maximum error in direction

:floats x4 return: minimum/maximum possible values of sin/cos of real direction
%}

theta = theta_nominal+theta_error_set;
theta(1)=mod(theta(1),2*pi);
theta(2)=mod(theta(2),2*pi);
sin_values = sin(theta);
cos_values = cos(theta);
min_sin=min(sin_values);
max_sin=max(sin_values);
min_cos=min(cos_values);
max_cos=max(cos_values);
if theta(1)>theta(2)
    max_cos=1;
end
if (theta(1)<pi/2)&&(theta(2)>pi/2)
    max_sin=1;
end
if (theta(1)<pi)&&(theta(2)>pi)
    min_cos=-1;
end
if (theta(1)<3*pi/2)&&(theta(2)>3*pi/2)
    min_sin=-1;
end
end