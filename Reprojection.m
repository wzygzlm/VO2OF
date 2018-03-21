function [X, Y, Z] = Reprojection(u, v, fx, fy, cx, cy, depth)
% Reprojection the image points back to real 3D points
X = (u*depth - cx)/fx;
X = (u*depth - cy)/fy;
Z = depth;
end

