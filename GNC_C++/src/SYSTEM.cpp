#include <Definitions.h>

struct timespec current_time;
double time_start;
double time_now;
double timer;
double timer_tic;

#define CLOCKID CLOCK_REALTIME
#define SIG SIGRTMIN

//Timer setup
uint8_t execute=0;
static void handler(int sig, siginfo_t *si, void *uc);
long long freq_nanosecs;
timer_t timerid;
struct sigevent sev;
struct itimerspec its;
struct timespec real_time_clock;
struct timespec real_time_clock_end;
sigset_t mask;
struct sigaction sa;

char reasons_msgs[7][10]={"System","Coms","RC","Sensors","Control","Failsafe","Unknown"};
char failsafe_msgs[3][10]={"Disarm","Land","Nothing"};

static void handler(int sig, siginfo_t *si, void *uc){
    execute=1;
}

void timer_begin(double delta_t){
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIG, &sa, NULL);
    sigemptyset(&mask);
    sigaddset(&mask, SIG);
    sigprocmask(SIG_SETMASK, &mask, NULL);
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIG;
    sev.sigev_value.sival_ptr = &timerid;
    timer_create(CLOCKID, &sev, &timerid);
    freq_nanosecs=(long long)(dt*1000000000);
    its.it_value.tv_sec = freq_nanosecs / 1000000000;
    its.it_value.tv_nsec = freq_nanosecs % 1000000000;
    its.it_interval.tv_sec = its.it_value.tv_sec;
    its.it_interval.tv_nsec = its.it_value.tv_nsec;
    timer_settime(timerid, 0, &its, NULL);
    sigprocmask(SIG_UNBLOCK, &mask, NULL);
}

void SYSTEM::begin(uint8_t new_limit){
    coms.send_msgs("Initilalising System...\n");
    limit=new_limit;
    
    //Set CPU at max frequency (Performance)
    if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)){           //Set CPU frequency at maximum
        coms.send_msgs("rc_set_cpu_freq failed \n");
    }
    sprintf(msgs,"CPU Frequency = %d MHz\n",rc_cpu_get_freq()/1000);
    coms.send_msgs();
    
    //Begin timer interrupt
    timer_begin((double)dt);
    
    //Set period_eps
    period_eps*=dt;
    
    //Get time
    clock_gettime(CLOCK_REALTIME,&current_time);
    time_start=current_time.tv_sec+(double)current_time.tv_nsec/1000000000.0f;
    timer=time_start;
    
    sprintf(msgs,"System Initialised! dt= %f\n",dt);
    coms.send_msgs();
}

void SYSTEM::update(){
    clock_gettime(CLOCK_REALTIME,&current_time);
    time_now=current_time.tv_sec+(double)current_time.tv_nsec/1000000000.0f;
    period=(float)(time_now-timer);
    timer=time_now;
    if(sync_time_request){
        time_start=time_now;
        sync_time_request=0;
    }
    Time=(float)(time_now-time_start);
    toc_func_counter=0;
}

uint8_t SYSTEM::check_period(){
    if(period>dt+period_eps||period<dt-period_eps)return false;
    return true;
}

uint8_t SYSTEM::check_exec_time(){
    clock_gettime(CLOCK_REALTIME,&current_time);
    time_now=current_time.tv_sec+(double)current_time.tv_nsec/1000000000.0f;
    exec_time=(float)(time_now-timer);
    if(exec_time>dt)return false;
    return true;
}

void SYSTEM::run_checks(uint8_t flag){
    //update LEDs
    if(armed){
        rc_led_set(RC_LED_GREEN, 1);
        rc_led_set(RC_LED_RED, 0);
    }else{
        rc_led_set(RC_LED_GREEN, 0);
        rc_led_set(RC_LED_RED, 1);
    }
    
    //Reset checks
    checks=0;
    
    //Check_period
    if(!check_period()){
        sprintf(msgs,"System: Sampling time violation (%f)!\n",period);
        coms.send_msgs();
    }
    
    //Check exec time
    if(!check_exec_time()){        
        sprintf(msgs,"System: Exec time violation (%f)!\n",exec_time);
        coms.send_msgs();
    }    
    
    //Print
    if(flag)print();
    
    //Reset and iterate toc
    if(toc_counter>=limit)toc_counter=0;
    toc_counter++;
}

uint8_t SYSTEM::arm_checks(){
    //Receiver or Joystick RC lock (Needs better checks!!!! 
    if(!receiver.active&&!coms.gcs_active)return true;
    else {
        if(coms.gcs_active&&receiver.force_disarm==false)return false;
        if(receiver.active&&!receiver.bounds&&!receiver.bounds2)return false;
    }
    return true;
}

void SYSTEM::arm(reasons reason){
    if(!armed){
        if(control.mode==RTL||control.mode==Land)return;
        if(!control.check_neutral_throttle())return;
        if(arm_checks())return;
        armed=1;
        sprintf(msgs,"Armed @ t=%.2f by %s\n",Time,&reasons_msgs[reason][0]);
        coms.send_msgs();
    }
}

void SYSTEM::disarm(reasons reason){
    if(armed){
        receiver.force_disarm=1;
        armed=0;
        sprintf(msgs,"Disarmed @ t=%.2f by %s\n",Time,&reasons_msgs[reason][0]);
        coms.send_msgs();
    }
}

void SYSTEM::log_msg(const char msg[]){
    char file_name[30]="/media/SD/Sys_Log.txt";
    if(datalog.mounted==1){
        if(!file_open){
            if(!file_created){
                f=fopen(file_name,"w");
            }
            else f=fopen(file_name,"a+");
            if(f!=NULL){
                file_open=1;
                file_created=1;
            }else return;
        }
        fprintf(f,"%s",msg);
        if(!armed){
            fclose(f);
            file_open=0;
        }
    }
}

void SYSTEM::print(){
    if(counter>=limit){
        counter=0;
        sprintf(msgs,"System State: Time = %f, Period = %f, Exec Time= %f, Checks = %d\n",Time,period,exec_time,checks);
        coms.send_msgs();
    }else counter++;
}

void SYSTEM::tic(){
    clock_gettime(CLOCK_REALTIME,&current_time);
    timer_tic=current_time.tv_sec+(double)current_time.tv_nsec/1000000000.0f;
}

float SYSTEM::toc(){
    clock_gettime(CLOCK_REALTIME,&current_time);
    time_now=current_time.tv_sec+(double)current_time.tv_nsec/1000000000.0f;
    float var=(float)(time_now-timer_tic);
    if(toc_counter>=limit){
        sprintf(msgs,"toc(%d) = %.6f\n",toc_func_counter,var);
        coms.send_msgs();
    }
    toc_func_counter++;
    return var;
}

void SYSTEM::sync_time(){
    //Reset main timer and time value
    sync_time_request=1;
    
    //Send coms
    coms.send_msgs("Time Synchronised!\n");
}

void SYSTEM::wait(float seconds){
    uint32_t N=round(seconds/dt);
    uint32_t microseconds=dt*1000000;
    for(uint8_t i=0;i<N;i++){
        usleep(microseconds);
    }
}

void SYSTEM::trigger_failsafe(failsafes failsafe){
    sprintf(msgs,"Failsafe %s triggered @ %f by %s\n",failsafe_msgs[failsafe],Time,reasons_msgs[Failsafe]);
    coms.send_msgs();
    switch(failsafe){

        case Disarm_fs:
        disarm(Failsafe);
        break;

        case Land_fs:
        control.set_mode(Land,Failsafe);
        break;

        case Do_Nothing:
        break;
    }
}