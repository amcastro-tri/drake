%time[s] #steps #xdot_evals #shrunk_by_errc #shrunk_by_fails #error_est_xdot_evals #jac_evals #matrx_facts

% ONLY SPHERE COLLISION

%acc = 1e-2;
data_ie =[11.0518 15670 165490 287  892  76440  626 2010]; %Implicit Euler.
data_r1 =[20.5168 32185 299833 339 1014 142166  779 2378]; % Radau1 %checked
data_r3 =[29.2774 30082 436681 393 1432 190166 1190 3046]; % Radau3
data_bs3=[2.35103  8391  36188 656    0     -1   -1   -1]; % Bogacky Shampine
data_rk3=[5.04224 19183  72825 5092   0     -1   -1   -1]; % RK3  %checked

data_1em2 = [data_ie; data_r1; data_r3; data_bs3; data_rk3];

%acc = 1e-3;
data_ie =[ 40.0667  60687  607021   705 394 327333 416 1458];
data_r1 =[ 13.5478  14802  190735   501 362 106324 365 1253]; %checked
data_r3 =[119.4580 128872 1814354   791 566 854369 508 1859];
data_bs3=[ 10.5892  30443  181952 15045 0 -1 -1 -1];
data_rk3=[  7.5202  31789  113997  6210 0 -1 -1 -1]; %checked

data_1em3 = [data_ie; data_r1; data_r3; data_bs3; data_rk3];

%acc = 1e-4;
data_ie = [ 62.5652  81869  895880  1018 217 531247 321 1172];
data_r1 = [ 19.0904  27202  264912  1018 204 115846 306 1144]; %checked
data_r3 = [314.7210 378183 4742418  1435 387 2024807 501 1769];
data_bs3= [ 17.8543  45344  312712 32834 0 -1 -1 -1];
data_rk3= [  8.37787 36635  127908  6001 0 -1 -1 -1]; %checked

data_1em4 = [data_ie; data_r1; data_r3; data_bs3; data_rk3];

%acc = 1e-5;
data_ie = [61.6190 81869 895880 1018 217 531247 321 1172];
%data_r1 = [17.8735 27202 264912 1018 204 115846 306 1144];
data_r3 = -ones(size(data_ie));
data_bs3= [17.9501 45344 312712 32834 0 -1 -1 -1];
data_rk3= [8.63026 38876 134169 5847 0 -1 -1 -1]; %checked


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BOX COLLISION INCLUDED

%acc = 1e-2;
%data_r1 =[]; chokes with > 3 min
data_rk3=[5.49109 34019 131544 9829 0 -1 -1 -1];

%acc = 1e-3;
data_rk3=[36.4181 234050 921912 73254 0 -1 -1 -1];

%acc = 1e-4;
%data_rk3=[]; % about 3 mins real for 1.1 secs sim time.




