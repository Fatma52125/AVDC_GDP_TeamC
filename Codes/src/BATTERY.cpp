#include <robotcontrol.h>
#include <BATTERY.h>
#include <COMS.h>
#include <SYSTEM.h>
#include <Types.h>
#include <Param.h>

extern COMS coms;
extern SYSTEM sys;

void BATTERY::begin(){
    coms.send_msgs("Initialising battery monitor...\n");
    
    //Load failsafes
    load_failsafes();
    
    //Initialise
    if(rc_adc_init()==-1)return;
    
    coms.send_msgs("Battery monitor initialised!\n");
}

void BATTERY::update(){
    //Read voltage
    if(!battery_monitor_type)voltage=(float)rc_adc_batt();
    else voltage=((float)rc_adc_read_volt(0))*voltage_scaler;
    filtered_voltage=alpha*filtered_voltage+(1-alpha)*voltage;
    in_range(&filtered_voltage,max_voltage,min_voltage);
    
    // sprintf(msgs, "Voltage = %f \n",voltage);
    // coms.send_msgs();
    //Read current
    current=((float)rc_adc_read_volt(1))*current_scaler;
    filtered_current=alpha*filtered_current+(1-alpha)*current;

    //Check failsafes
    check_failsafes();
}

void BATTERY::load_failsafes(){
    //Declare variable
    float parameters[10];
    
    //Read parameters
    save_read_param_float(file_name,fs_index,parameters,10,0);
    
    //Update
    update_failsafes(parameters);
}

void BATTERY::save_failsafes(float parameters[]){
    //Update
    if(!update_failsafes(parameters))return;
    
    //Save if all ok
    save_read_param_float(file_name,fs_index,parameters,10,1);
}

uint8_t BATTERY::update_failsafes(float parameters[]){
    //Check input
    float norm=0;
    for(uint8_t i=0;i<10;i++)norm+=parameters[i];
    if(norm<=eps){
        coms.send_msgs("Battery: Error updating parameters!\n");
        return false;
    }
    // n_fs=2;
    //Copy if all ok
    // uint16_t fs_temp[n_fs];
    // for(uint8_t i=0;i<n_fs;i++)fs_temp[i]=(uint16_t)parameters[i];
    // memcpy(fs,fs_temp,n_fs*2);
    // memcpy(fs_lim,parameters+n_fs,n_fs*4);
    fs_set=1;

    //Set battery type
    // battery_monitor_type=(uint8_t)round(parameters[5]);
    fs[0] = Land_fs;
    fs[1] = Disarm_fs;
    fs_lim[0] = 6.5f;
    fs_lim[1] = 3.0f;
    battery_monitor_type=0;
    //Print
    print_failsafes();
    
    //Return
    return true;
}

void BATTERY::print_failsafes(){
    coms.send_msgs("Battery Failsafes:\n");
    for(uint8_t i=0;i<n_fs;i++){
        sprintf(msgs,"Failsafe %d: %s @ V = %.2f\n",i,failsafe_msgs[fs[i]],fs_lim[i]);
        coms.send_msgs();
    }
    sprintf(msgs, "Battery Monitor Type = %d\n",battery_monitor_type);
    coms.send_msgs();
}

void BATTERY::check_failsafes(){
    if(!fs_set)return;
    uint8_t flag=0;
    for(uint8_t i=0;i<n_fs;i++){
        if(voltage<fs_lim[i]){
            if(!fs_triggered[i]){
                coms.send_msgs("Battery below limit!\n");
                status=0;
                fs_triggered[i]=1;
                sys.trigger_failsafe(fs[i]);
                return;
            }
            flag=1;
        }
    }
    if(!flag)reset_failsafes();
}

void BATTERY::reset_failsafes(){
    status=1;
    for(uint8_t i=0;i<n_fs;i++){
        fs_triggered[i]=0;
    }
}