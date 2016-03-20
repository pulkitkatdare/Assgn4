%code for Adabooost algorithm
clear 
dataset = normrnd(0,2,2000,2);
dataset_A = zeros(1,2);
count = 0;
for i = 1:2000
if(norm(dataset(i,:)) < 2)
count = count + 1 ; 
dataset_A(count,:) =dataset(i,:);
end
if((norm(dataset(i,:)) < 3) & (norm(dataset(i,:)) > 2.5) )
count = count + 1 ; 
dataset_A(count,:) =dataset(i,:);
end
end 
plot(dataset(:,1),dataset(:,2),'+')
hold on 
plot(dataset_A(:,1),dataset_A(:,2),'+')
%&(dataset(2,:) > 0.3 )& (dataset(2,:) < 0.7 ))
%dataset_B = 0dataset((dataset(:,1) > 0.7 | dataset(:,1) < 0.3) & (dataset(:,2) > 0.7 | dataset(:,2) < 0.3) ); 



%plot(dataset_A(:,1),dataset_A(:,2),'+','b');
%hold on 
%plot(dataset_B(:,1),dataset_B(:,2),'+','r');
