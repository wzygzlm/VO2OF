% This example is used to verify the jacobian matrix in the BCH formula
% We used two methods to calculate the difference between two se3 values.
% One method is projecting the difference of their corresponding SE3 back
% to se3 space without jacobian.
% The other method is projecting with jacobian.
% Result shows that if the basic se3 value is small, then these two methods
% have similar result. However, if  the basic se3 value is very big, here
% we amplify it with 500, then the result will differ a lot. 
% The reason is when the basic se3 is very small, then jacobian is close to
% eye.

se1 = [0, 0, 0, 0.001, 0.001, 0.001];
se1_amplify = se1 * 500; % Change the amplify value to observe the different result
delta_se = [0.00, 0.00, 0.00, 0.02, 0.01, 0.03];

%% 
disp('The original delta se3 is:')
disp(delta_se)

%%
disp('Delta se3 calculated by projecting SE3 back to se3 without jacobian matrix')
delta_se_without_jaco = SE3_se3_back(se3_SE3(se1_amplify + delta_se) * se3_SE3(-se1_amplify));
disp(delta_se_without_jaco);

%%
[origin_se3 jacobian] = SE3_se3_back(se3_SE3(se1_amplify));
jacobian
disp('Delta se3 calculated by projecting SE3 back to se3 with jacobian matrix')
delta_se_with_jaco = jacobian * delta_se_without_jaco(4:6)';
disp([delta_se_without_jaco(1:3) delta_se_with_jaco']);