%code for Adabooost algorithm
clear
format longe
load data1
s = RandStream('mt19937ar','Seed',0);
points=randperm(s,2000,1000);%randi([1 2000],1000,1);
points_left =setdiff(1:2000,points);
m = length(points_left);
points_left=randi([1,m],1000,1);
train_data = dataset(points_left,:);
test_data = dataset(points,:); 
check_data1 = [test_data(:,1);test_data(:,2)];
check_data2 = [ test_data(:,2)] ;
y_t = zeros(1000,1);
for k =1:1000
    if((test_data(k,1) < 0.7) & ((test_data(k,1) > 0.3)) & (test_data(k,2) < 0.7) & ((test_data(k,2) > 0.3)))
        y_t(k) = +1;
    else
        y_t(k) = -1;
    end
end
y_t1 = zeros(1000,1);
for k =1:1000
    if((train_data(k,1) < 0.7) & ((train_data(k,1) > 0.3)) & (train_data(k,2) < 0.7) & ((train_data(k,2) > 0.3)))
        y_t1(k) = +1;
    else
        y_t1(k) = -1;
    end
end

%test_data = [dataset(points,:)]; 
%check_data1 = [test_data(:,1);test_data(:,2)];
%check_data2 = [ test_data(:,2)] ;

T = 40;
epsilon = zeros(T,1) ;
weights = 10e-4*ones(1000,1);
weights_test = 10e-4*ones(1000,1);
weights_check = weights;
alpha_t = zeros(T,1);
p = zeros(T,1);
theta = zeros(T,1);
i = ones(T,1);
sum1 = zeros(1000,1);
sum2 = zeros(1000,1);
delta= zeros(T,1);
delta1 = zeros(T,1);
for j = 1:T
    fprintf('no of iterations left =%d.\n',T-j)
    m = length(check_data1);
    m = randi([1,m]);
    angle = check_data1(m);
    theta(j) = angle;
    [q,w] = find(test_data==angle);
    i(j) = w(1);
    ptest = -1 ;
    pred1 =zeros(1000,1);
    err = 0 ;
    for e=1:1000
        if (sign(ptest*(test_data(e,w)-angle))<=0)
            pred1(e) = -1;
        else
            pred1(e)  =1 ;
        end
        if (y_t(e) ~= pred1(e))
            err = err + weights(e);
        end
    end
    if (err > 0.50)
        err = 1-err;
        ptest = -ptest;
    end
    errmin = err;
    p(j) = ptest;
    pred2 = pred1;
    m_check = m ;
    check_data1 = [check_data1(1:m_check-1,1);check_data1(m_check+1:end,1)];
    check = weights;
    h_t = zeros(1000,1);
    h_t = pred2;
    epsilon(j) = errmin;
    alpha_t(j) = 0.5*log((1-epsilon(j))/epsilon(j));
    Z_t = 0;
    Z_t1 =0;
    for k = 1:1000
        Z_t =double(Z_t + weights(k)*exp(-alpha_t(j)*y_t(k)*h_t(k,1)));
        weights(k) = double(weights(k)*exp(-alpha_t(j)*y_t(k)*h_t(k,1)));
    end
    weights = weights/(Z_t+10e8*eps);
    %weights_test = weights_test/(Z_t+10e8*eps);
    for l = 1:1000
        l;
        sum1(l) = sum1(l) + alpha_t(j)*(p(j)*(test_data(l,i(j))-theta(j)));
        %sum2(l) = sum2(l) + alpha_t(j)*(p(j)*(train_data(l,i(j))-theta(j)));
        if(sum1(l) <=0)
            pred = -1;
        else
            pred =1;
        end
        if ( y_t(l) ~= pred)
            delta(j) = delta(j) + weights(l);
        end
        
%sum2(l) = sum2(l) + alpha_t(j)*(p(j)*(train_data(l,i(j))-theta(j)));
        %if((sum2(l)) <=0)
         %   pred = -1;
        %else
         %   pred = 1;
        %end
        %if ( y_t1(l) ~= pred)
         %   delta1(j) = delta1(j) + weights_test(l);
        %end
        
    end
    %if ( delta1(j) > 0.5 )
    %delta1(j) = 1-delta1(j);
    %end
    
end
%% PLOTS THE DATA
n = 1:T;
figure
plot(n',delta);
figure
plot(dataset(:,1),dataset(:,2),'+')
hold on
plot(dataset_A(:,1),dataset_A(:,2),'+')
hold on
for j = 1:T
    if(i(j) ==1)
        plot([theta(j) theta(j)],[0 1]);
    else
        plot([0 1],[theta(j) theta(j)]);
    end
end