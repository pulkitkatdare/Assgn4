%code for Adabooost algorithm
clear
format longe
load data2

test_data = dataset(1:1000,:);
y_t = zeros(1000,1);
for k =1:1000
if((test_data(k,1) < 0.7) & ((test_data(k,1) > 0.3)) & (test_data(k,2) < 0.7) & ((test_data(k,2) > 0.3)))
y_t(k) = +1;
else 
y_t(k) = -1;
end
if(test_data(k,1) < 0.25 & test_data(k,1) > 0.15)
y_t(k) = +1;
end
if(test_data(k,1) < 0.85 & test_data(k,1) > 0.75)
y_t(k) = +1;
end
if(test_data(k,2) < 0.85 & test_data(k,2) > 0.75)
y_t(k) = +1;
end
if(test_data(k,1) < 0.25 & test_data(k,1) > 0.15)
y_t(k) = +1;
end
end

T = 40; 
epsilon = zeros(T,1) ;
weights = 10e-4*ones(1000,1);
alpha_t = zeros(T,1);
p = zeros(T,1);
theta = zeros(T,1);
i = ones(T,1);
sum1 = zeros(1000,1);
delta= zeros(T,1);
for j = 1:T
	j
    errmin = 10;
    err = zeros(1000,2);
for q = 1:1000
    for w = 1:2
    angle = test_data(q,w);
    %i(j)  = w;
    ptest = 1 ;
     pred1 =zeros(1000,1);
    for e=1:1000
        if ((ptest*(test_data(e,w)-angle))<=0)
        pred1(e) = -1;
        else 
            pred1(e)  =1 ;
        end
    if (y_t(e)~=pred1(e))
    err(q,w) = err(q,w) + weights(e);
    end
    end
if (err(q,w) > 0.50)
err(q,w) = 1-err(q,w);
ptest = -ptest;
end
if(err(q,w) < errmin)
errmin = err(q,w);
theta(j) = angle;
i(j) = w;
pred2 = pred1;
p(j) = ptest;
end


end
end
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
h_t = zeros(1000,1);

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
for k = 1:1000
Z_t =double(Z_t + weights(k)*exp(-alpha_t(j)*y_t(k)*h_t(k,1)));
weights(k) = double(weights(k)*exp(-alpha_t(j)*y_t(k)*h_t(k,1)));
end
weights = weights/(Z_t+10e8*eps);
%weights = weights/sum(weights);
%weights = weights/sum(weights);

%digits(10);
%weights = vpa(weights);
for l = 1:1000
        l;
    sum1(l) = sum1(l) + alpha_t(j)*(p(j)*(test_data(l,i(j))-theta(j)));
    if(sum1(l) <=0)
    pred = -1;
    else
    pred =1;
    end
    if ( y_t(l) ~= pred)
    delta(j) = delta(j) + weights(l);
    end
end      
    
end

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