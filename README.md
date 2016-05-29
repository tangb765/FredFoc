# FredFoc
git clone https://github.com/FredDeng2016/FredFoc.git
git submodule init
git submodule update

git branch

git submodule add https://github.com/RT-Thread/rt-thread.git RT-Thread

git status

git add -u
git commit -m "add RT Thread submodule"

git push

git pull

#system command list

LOCK_MODE = 0,
YAW_FOLLOW_MODE,
YAW_AND_ROLL_FOLLOW_MODE,
INVERSION_LOCK_MODE,
INVERSION_YAW_FOLLOW_MODE,
INVERSION_YAW_AND_ROLL_FOLLOW_MODE,
MODE_NUMS,		



#E,8916,8917,8918,8919,8920,8921$

#E,8916,8917,8918,8919,8920,8921@

#E,0,8917,0,0,0,0@

#E,1000,0,0,0,0,0@

#E,4000,0,0,0,0,0@

#E,4000,0,0,100,0,0@

#E,15000,0,0,10,0,0@


#E,200,0,0,2500,0,0@

#save,system,configs@

#console,a@
#console,b@
#console,c@
#console,null@
#console,all@

#imu,open@
#imu,close@
#imu,gyro,calib@
#imu,acc,calib@
#imu,acc,calib,0,0,0,1,0,0,0,1,0,0,0,1@

#imu,get,state@
#imu,get,gyro,bias@
#imu,get,acc,bias@
#imu,get,acc,martixA@



#motor,run@
#motor,align@
#motor,calilevel@
#motor,stop@   
#motor,test,speed@
#motor,test,iqcurrent@        
#motor,test,idcurrent@      
#motor,test,close@


//串口打印调试数据

#test,close@
#test,tle5012@
#test,mpu6500@
#test,motor@

#get,align@
#get,calilevel@

#get,pid,pk@
#get,pid,pi@
#get,pid,pd@

#get,pid,sk@
#get,pid,si@
#get,pid,sd@


#get,pid,tk@
#get,pid,ti@
#get,pid,td@

#get,pid,fk@
#get,pid,fi@
#get,pid,fd@



//pid参数调试
#pid,tk,2470,4096@
#pid,ti,3238,8192@
#pid,td,100,8192@

#pid,fk,2470,4096@
#pid,fi,3238,8192@
#pid,fd,100,8192@

#pid,sk,8000,64@  
#pid,si,800,1024@
#pid,sd,0,16@

#pid,pk,52,512@
#pid,pi,0,16384@
#pid,pd,0,512@


//测试
#console,null@
#console,a@
#test,motor@
#motor,align@
#motor,calilevel@
#save,system,configs@

//FLUX测试
#imu,close@
#console,a@
#test,motor@
#motor,test,idcurrent@ 
#E,200,0,0,2000,0,0@
#E,0,200,0,0,2000,0@
#E,0,0,200,0,0,2000@
#motor,run@
#motor,stop@ 


#pid,fi,1100,8192@
#pid,fk,700,4096@

//力矩测试
#imu,close@
#console,a@
#test,motor@
#motor,test,iqcurrent@ 
#E,200,0,0,2000,0,0@
#E,0,200,0,0,2000,0@
#E,0,0,200,0,0,6000@
#motor,run@
#motor,stop@ 


#pid,ti,1100,8192@
#pid,tk,700,4096@


//速度测试
#imu,open@
#console,a@
#test,motor@                                                                                                
#motor,test,speed@                                                                                     
#E,5000,0,0,50,0,0@  
#E,0,5000,0,0,50,0@ 
#E,0,0,5000,0,0,50@ 
#E,20000,0,0,1000,0,0@    
#E,20000,0,0,90,0,0@  
#E,5000,0,5000,50,0,100@         
#motor,run@   
#motor,stop@ 


#pid,sk,4800,64@
#pid,si,1000,1024@

//位置测试
#imu,close@
#console,a@
#test,motor@ 
#motor,test,close@
#E,5000,0,0,50,0,0@  
#E,0,5000,0,0,50,0@ 
#E,0,0,5000,0,0,50@ 
#motor,run@  
#motor,stop@ 

#pid,pk,52,512@
#pid,pi,0,16384@
#pid,pd,0,512@

//关    
#motor,stop@           
