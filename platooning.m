clear
myev3=legoev3('usb');
mymotor1=motor(myev3,'B');
mymotor2=motor(myev3,'C');
mdiff=3;
mymotor1.Speed=mdiff;
mymotor2.Speed=0;
start(mymotor1);
start(mymotor2);
j=1;
kp=0.1;
fid = fopen('ECE556-group13_task3_trial1.txt','w');         %save data to file
start_t = clock;
Start_conv=start_t(6)+60*start_t(5)+60*60*start_t(4);       
us_prev=0;      
while (true)
    mdiff=3;
    us_read=readInputDeviceREADY_RAW(myev3,4,0,1);
    us = us_read/10;
    us_array(j)=us;
    us_diff = us-us_prev;
    if us-21>5                                              %check if the block is in rnage
        K_sep=13;
        ctrl=5;
        K_TA=0.3;
    elseif us-21<5                                          %set speed accordingly
        K_sep=0;
        ctrl=0;
        K_TA=8;
        mdiff = 0;
    else
        K_sep = 10;
        ctrl=us-20;
        K_TA = 1;
    end
    Speed_ctrl = (K_sep*ctrl) + (K_TA* us_diff);            %set speed
    ls_read=readInputDeviceREADY_RAW(myev3,2,0,1);%2988 3424
    read_values_normal=(ls_read-2950)/5;
    error=(read_values_normal-50);
    ls_read_l=readInputDeviceREADY_RAW(myev3,1,0,1)%2988 3424
    if ls_read_l>3300
        mymotor1.Speed=0;
        mymotor2.Speed=0;
        break
    end
    if(error>0)
        mymotor1.Speed=Speed_ctrl+mdiff;
        mymotor2.Speed=Speed_ctrl+kp*ctrl*(error);
    else
        mymotor1.Speed=Speed_ctrl-kp*ctrl*(error)+mdiff;
        mymotor2.Speed=Speed_ctrl;
    end
    us_prev=us;
    Time_st = clock;
    Time_conv=Time_st(6)+60*Time_st(5)+60*60*Time_st(4);
    T_stamp(j)=Time_conv-Start_conv;
    j=j+1;
end
tq_t3=0.05:0.05:0.05*length(T_stamp)+0.05;
T_stamp(length(T_stamp)+1)=T_stamp(length(T_stamp))+0.05;
vq=zoh(T_stamp',us_array',tq_t3',false);
for l=1:length(vq)
    fprintf(fid,'@ %f @ %f\r\n',vq(l),tq_t3(l));
end
fclose(fid);
