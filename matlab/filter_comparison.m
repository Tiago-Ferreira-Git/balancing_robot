clear all;
close all;
device =  serialport("COM8",57600);


x = zeros(40000,10);

i = 1;



while 1
    if device.NumBytesAvailable > 0
        readline(device);
        data = split(readline(device)); 
        data = data(2:11);
        data = str2double(data');
        data;
        x(i,:) = data;
        i = i + 1;
        if(i == 40000)
            break
        end

    end    
end

%%

figure
hold on
stairs(x(1:end-1,1),x(1:end-1,4))
stairs(x(1:end-1,1),x(1:end-1,5))
stairs(x(1:end-1,1),x(1:end-1,6))
stairs(x(1:end-1,1),x(1:end-1,7))
stairs(x(1:end-1,1),x(1:end-1,8))
stairs(x(1:end-1,1),x(1:end-1,9))
stairs(x(1:end-1,1),x(1:end-1,10))
legend({"Measured","Filter n = 3","Filter n = 5","Filter n = 10","Filter n = 15","Filter n = 20","Kalman"})

%%
mask = bitand((11+1e-3 <= x(:,1)), (x(:,1) <= 11+1e-3));  

index_1 = find(mask == 1);
index_1 = index_1(1);

mask = bitand((15-1e-3 <= x(:,1)), (x(:,1) <= 15+1e-3));  

index_2 = find(mask == 1);
index_2 = index_2(1);


n = [3,5,10,15,20];
sprintf("Mean %f STD %f Measured",mean(x(index_1:index_2,4)),std(x(index_1:index_2,4)))
for i = 5:9

    figure
    hold on
    stairs(x(index_1:index_2,1),x(index_1:index_2,4))
    stairs(x(index_1:index_2,1),x(index_1:index_2,i))
    legend({"Measured",sprintf("Filter n = %d",n(i-4))})

    sprintf("Mean %f STD %f Filter %d",mean(x(index_1:index_2,i)),std(x(index_1:index_2,i)),n(i-4))


end

mask = bitand((31+1e-3 <= x(:,1)), (x(:,1) <= 31+1e-3));  

index_1 = find(mask == 1);
index_1 = index_1(1);

mask = bitand((35-1e-3 <= x(:,1)), (x(:,1) <= 35+1e-3));  

index_2 = find(mask == 1);
index_2 = index_2(1);


n = [3,5,10,15,20];
sprintf("Mean %f STD %f Measured",mean(x(index_1:index_2,4)),std(x(index_1:index_2,4)))
for i = 5:9

    figure
    hold on
    stairs(x(index_1:index_2,1),x(index_1:index_2,4))
    stairs(x(index_1:index_2,1),x(index_1:index_2,i))
    legend({"Measured",sprintf("Filter n = %d",n(i-4))})

    sprintf("Mean %f STD %f Filter %d",mean(x(index_1:index_2,i)),std(x(index_1:index_2,i)),n(i-4))


end