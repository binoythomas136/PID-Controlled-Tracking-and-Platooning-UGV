clear
clc
myev3=legoev3('usb');                           //connect via usb
set_speed=30;                                   //set the base speed to 30
mdiff=5;                            
mymotor1=motor(myev3,'B');                      //connect motor to port B
mymotor2=motor(myev3,'C');                      //connect the other motor to port C
mymotor1.Speed=set_speed+mdiff;                 //set speed
mymotor2.Speed=set_speed;
kp=1.8;                                         %set kp 
ki=0.08;                                        %set integral component
kd= 5;%8;                                       %set differential component
prev_error=0;
start(mymotor1);
start(mymotor2);
i=1;
fid = fopen('ECE556-group13_task1_trial1.txt','w'); %store data in file
start_t = clock;
Start_conv=start_t(6)+60*start_t(5)+60*60*start_t(4);
k=1;
j=1;
while (true)
    ls_read=readInputDeviceREADY_RAW(myev3,2,0,1);%2988 3424 %get snesore values
    us_read=readInputDeviceREADY_RAW(myev3,4,0,1);          %get ultrasonic values
    us = us_read/10;
    read_values_normal=(ls_read-2950)/5;                    %normaliza values
    error=(read_values_normal-50);  
    error_list(i)=error;
    sensor_readings(k)=read_values_normal;
    sum_e=sum(error_list);                                  %sum the previous values to get integral term
    diff=error-prev_error;                                  %find difference for differental term
        if(us<8)
            mymotor2.Speed=0;
            mymotor1.Speed=0;
            break
        end
        if(error>0)
            mymotor2.Speed=set_speed;
            mymotor1.Speed=set_speed+kp*(error)+kd*(diff)+ki*(sum_e)+mdiff;%change speed of mototr to follow
        else
            mymotor2.Speed=set_speed-kp*(error)-kd*(diff)-ki*(sum_e);%change speed of motor to follow
            mymotor1.Speed=set_speed+mdiff;
        end
    Time_st = clock;
    Time_conv=Time_st(6)+60*Time_st(5)+60*60*Time_st(4);
    T_stamp(k)=Time_conv-Start_conv;
    prev_error=error;
    if(i==5)
    i=0;
    end
    i=i+1;
    k=k+1;
    end
tq_t1=0.05:0.05:0.05*length(T_stamp)+0.05;
T_stamp(length(T_stamp)+1)=T_stamp(length(T_stamp))+0.05;
vq=zoh(T_stamp',sensor_readings',tq_t1',false);
for l=1:length(vq)
    fprintf(fid,'@ %f @ %f\r\n',sensor_readings(l),tq_t1(l));   %save data to file
end
fclose(fid);
