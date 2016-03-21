%code for MNIST
%{
clear 
load MNIST
label_testdata = labels(1:5000) ;
I_testdata = zeros(5000,28*28);
for i= 1:5000
I_testdata(i,:) = reshape(I{i},[1,28*28]);
if (labels(i) == 2)
label_testdata(i) = 1;
else
label_testdata(i) = -1;    
end
end
%}
%%% The code above is for labelling the data and converting it into 28*28
%%% format column vector ;
%%
clear
load MNIST1
y_t = double(label_testdata);
check_data1 = reshape(I_testdata,[5000*784,1]);
T = 40 ; %no of iterations
epsilon = zeros(T,1) ;
weights = (1/5000)*ones(5000,1);%weights of the individual training data;
alpha_t = zeros(T,1);
p = zeros(T,1);
theta = zeros(T,1);
i = zeros(T,1);
sum1 = zeros(5000,1);
delta= zeros(T,1);
for j = 1:T
	j
    errmin = 10;
    %err = zeros(1000,2);
 %   if(rem(j,2) ==0)
m = length(check_data1);
  %  else
   % m = length(check_data2);
   % end
%for prod = 1:m
    m = randi([1,m]);
    angle = check_data1(m);
    theta(j) = angle;
    [q,w] = find(I_testdata==angle);
    %i(j)  = w;
    i(j) = w(1);
    ptest = -1 ;
    pred1 =zeros(5000,1); 
    err = 0 ;
    for e=1:5000
        e
        if ((ptest*(I_testdata(e,w)-angle))<=0)
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
%if(err < errmin)
%errmin = err;
%theta(j) = angle;
%q_check = q;
%m_check = m;
%i(j) = w(1);
%p(j) = ptest;
%pred2 = pred1;
%end
%end
fprintf('checkpoint1');

%check_data(q_check*i(j))= 0.5;
%if (rem(j,2) == 0)
check_data1 = [check_data1(1:m_check-1,1);check_data1(m_check+1:end,1)];
%else
 %   check_data2 = [check_data2(1:m-1,1);check_data2(m+1:end,1)];
%end
check = weights;    
%theta(j) = rand(1);
%p(j) = 1; 
%poss1 = sum(weights.*(-y_t+test_data(:,1)))/sum(weights);
%poss2 = sum(weights.*(-y_t+test_data(:,2)))/sum(weights);
%if (sum(weights.*(y_t-(test_data(:,1)-poss1)).^2) < sum(weights.*(y_t-(test_data(:,2)-poss2)).^2))
%theta(j) = poss1;
%i(j) = 1;
%else
%theta(j) = poss2;
%i(j) = 2;
%end
h_t = zeros(5000,1);

%for k = 1:1000
%if (p(j)*(test_data(k,i(j))-theta(j)) <=0)

%	h_t(k) = 1;
%else
	%h_t(k) = -1;
%end
	
%h_t(k,1) = sign(p(j)*(test_data(k,i(j))-theta(j)));


%if(y_t(k) ~= h_t(k))
%epsilon(j) = epsilon(j) + weights(k);
%end

%end
h_t = pred2;
epsilon(j) = errmin;
%if (epsilon(j) > 0.5)
%p(j) = -p(j);
%h_t = -h_t;
%theta(j)=-theta(j);
%epsilon(j) = 1- epsilon(j);
%end
alpha_t(j) = 0.5*log((1-epsilon(j))/epsilon(j));

Z_t = 0;
for k = 1:5000
Z_t =double(Z_t + weights(k)*exp(double(-alpha_t(j)*y_t(k)*h_t(k,1))));
weights(k) = double(weights(k)*exp(double(-alpha_t(j)*y_t(k)*h_t(k,1))));
end
weights = weights/(Z_t+10e8*eps);
%weights = weights/sum(weights);
%weights = weights/sum(weights);

%digits(10);
%weights = vpa(weights);
fprintf('checkpoint2');
for l = 1:5000
        l;
    sum1(l) = sum1(l) + alpha_t(j)*(p(j)*(I_testdata(l,i(j))-theta(j)));
    if(sum1(l) <=0)
    pred = -1;
    else
    pred =1;
    end
    if ( y_t(l) ~= pred)
    delta(j) = delta(j) + weights(l);
    end
end      
  delta(j)   
end