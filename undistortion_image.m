clear;
close all;
tangentialDistortion = [-0.058567490089618   0.037904697518710];
radialDistortion = [-0.034281894022168  -0.008244485612567]; 
% tangentialDistortion = [-0.001774514973451,6.230176254781805e-05];
% radialDistortion = [-0.283675926885368,0.088238919987596,7.973409664780424e-04]; 
IntrinsicMatrix = 1.0e+02 * [
                               2.257028124771077                   0                   0
                                               0   2.253928747331328                   0
                               1.673925764568589   1.268120194904375   0.010000000000000];

%20180316
% tangentialDistortion = [-7.765323851662319e-04,-8.916400158462725e-04];
% radialDistortion = [-0.285401311137702,0.084124563710154,0.013197090097920];
% IntrinsicMatrix = [112.933732108099	0	0
% 		    -0.263999912135936	113.053937283519	0
% 		    75.2882322927443	66.4730617361768	1];

cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',radialDistortion,'TangentialDistortion',tangentialDistortion);
cameraParams.ImageSize = [260, 346];
I = imread('/media/minliu/dataset/MVSEC/ApsFrame-2018-04-17T13-28-49+0200.png');
[J1, newOrigin] = undistortImage(I,cameraParams);
figure; imshowpair(I,J1,'montage');
title('Original Image (left) vs. Corrected Image (right)');title( 'Origin image' );
J2 = undistortImage(I,cameraParams,'OutputView','full');
figure;
imshow(J2);
title('Full Output View');