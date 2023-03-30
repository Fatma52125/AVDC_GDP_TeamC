#include <Definitions.h>

void RECEIVER::begin(uint8_t new_limit){
    coms.send_msgs("Initializing receiver...\n"); 
    limit=new_limit;

    //Load RC UART
    uart.begin(uart_bus,uart_baudrate,uart_timeout,0,1,0);
    sprintf(msgs,"RC UART: %d @ %d\n",uart_bus,uart_baudrate);
    coms.send_msgs();
    
    //Load calibration
    load_or_save_calibration(0);

    //Load failsafes
    load_or_save_failsafes(0);
    
    coms.send_msgs("Receiver Initialized!\n");
    sleep(1);
}

void RECEIVER::set_offsets_and_scales(){
    //Set receiver chanels with angle type of operation
    set_angle_ch(roll_ch,500,0.05);
    set_angle_ch(pitch_ch,500,0.05);
    set_angle_ch(yaw_ch,500,0.05);
    
    //Set receiver chanels with throttle type of operation
    set_throttle_ch(throttle_ch,1000,0.05);
    set_throttle_ch(enable_ch,1000,0.05);
    set_throttle_ch(mode_ch,1000,0.05);
    set_throttle_ch(aux1_ch,1000,0.05);
    set_throttle_ch(aux2_ch,1000,0.05);
    
    //Print
    print_offsets_and_scales();
}

void RECEIVER::load_or_save_calibration(uint8_t save){
    uint16_t parameters[n_ch*2];
    if(!save){
        save_read_param_uint16(file_name,limits_index,parameters,2*n_ch,0);
        memcpy(upper_limits,parameters,sizeof(upper_limits));
        memcpy(lower_limits,parameters+n_ch,sizeof(lower_limits));
    }
    uint8_t flag=check_calibration();
    if(flag||save){
        memcpy(parameters,upper_limits,sizeof(upper_limits));
        memcpy(parameters+n_ch,lower_limits,sizeof(lower_limits));
        save_read_param_uint16(file_name,limits_index,parameters,2*n_ch,1);
    }
    set_offsets_and_scales();
}

uint8_t RECEIVER::check_calibration(){
    uint8_t flag=false;
    
    //Check channel limits
    for(uint8_t i=0;i<active_ch;i++){
        if(upper_limits[i]>2100||upper_limits[i]<1900)flag=true;
        if(lower_limits[i]<900||lower_limits[i]>1100)flag=true;
    }
    
    //Reset if flag true
    if(flag){
        coms.send_msgs("Receiver calibration problem! Resetting all channels...\n");
        for(uint8_t i=0;i<active_ch;i++){
            upper_limits[i]=2000;
            lower_limits[i]=1000;
        }
        receiver_cal=0;
    }else receiver_cal=1;
    
    //Print resulting calibration
    print_calibration();
    
    return flag;
}

void RECEIVER::print_calibration(){
    coms.send_msgs("Receiver calibration...\n");
    coms.send_msgs("Max | Min\n");
    for(uint8_t i=0;i<active_ch;i++){
        sprintf(msgs,"%d|%d\n",upper_limits[i],lower_limits[i]);
        coms.send_msgs(msgs);
    }
}

void RECEIVER::calibrate(){
    //Reset calibration
    uint16_t upper_limits2[active_ch];
    uint16_t lower_limits2[active_ch];
    for(uint8_t i=0;i<active_ch;i++){
        upper_limits[i]=2500;
        lower_limits[i]=500;
        upper_limits2[i]=1500;
        lower_limits2[i]=1000;
    }
    
    //Prompt user inputs
    coms.send_msgs("Move RC to all limits and give input when ready.\n");
    while(!coms.any_message_available()){
		update(0);
        for(uint8_t i=0;i<active_ch;i++){
            if(raw_channels[i]>upper_limits2[i])upper_limits2[i]=raw_channels[i];
            if(raw_channels[i]<lower_limits2[i])lower_limits2[i]=raw_channels[i];
        }
        if(counter>limit){
            counter=0;
            coms.send_msgs("Upper|Lower\n");
            for(uint8_t i=0;i<active_ch;i++){
                sprintf(msgs,"%d|%d\n",upper_limits2[i],lower_limits2[i]);
                coms.send_msgs();
            }
        }else counter++;
        usleep(20000);
    }
    memcpy(upper_limits,upper_limits2,active_ch*2);
    memcpy(lower_limits,lower_limits2,active_ch*2);
    
    //Save
    load_or_save_calibration(1);
}

void RECEIVER::set_angle_ch(uint8_t i,float range, float dead_zone){
    float bias=(float)((upper_limits[i]+lower_limits[i])/2.0f);
    float scale=(float)((upper_limits[i]-lower_limits[i])/2.0f);
    offsets[i]=bias;
    scales[i]=range/scale;
    dead_zones[i]=range*dead_zone;
    ch_max[i]=range;
    ch_min[i]=-range;
}

void RECEIVER::set_throttle_ch(uint8_t i, float range, float dead_zone){
    float scale=(float)((upper_limits[i]-lower_limits[i]));
    offsets[i]=(float)(lower_limits[i]);
    scales[i]=range/scale;
    dead_zones[i]=range*dead_zone;
    ch_max[i]=range;
    ch_min[i]=0;
}

void RECEIVER::print(){
    if(counter>limit){
        counter=0;
        coms.send_msgs("RC Channels: \t");
        coms.print_matrix(channels,1,active_ch,0);
    }else counter++;
}

void RECEIVER::print_raw(){
    if(counter>limit){
        counter=0;
        coms.send_msgs("RC Raw Channels: \t");
        for(uint8_t i=0;i<active_ch;i++)printf("%hu, ",raw_channels[i]);
        printf("\n");
    }else counter++;
}

void RECEIVER::print_offsets_and_scales(){
    coms.send_msgs("Offsets|Scales\n");
    for(uint8_t i=0;i<active_ch;i++){
        sprintf(msgs,"%.1f|%.1f\n",offsets[i],scales[i]);
        coms.send_msgs();
    }
}

void RECEIVER::offset_channels(){
    if(active){
        for(uint8_t i=0;i<active_ch;i++){
            channels[i]=scales[i]*((float)raw_channels[i]-offsets[i]);
        }
    }
}

void RECEIVER::cut_buffer(uint16_t i){    
    memmove(buffer, buffer+i, index-i+1);
    index-=i;
}

uint8_t RECEIVER::validate_iBUS_checksum(){
    //Compute checksum
    uint16_t chksm=0xFFFF;
    uint8_t array[28];
    memcpy(array,&iBUS,28);
    for(uint8_t i=0;i<28;i++){
        chksm-=array[i];
    }
    chksm-=0x20;
    chksm-=0x40;

    //Validate checksum
    if(chksm==iBUS.checksum){
        memcpy(raw_channels,&iBUS,2*active_ch);
        return true;
    }else{
        coms.send_msgs("RECEIVER: IBUS checksum error");
        return false;

    }
}

void RECEIVER::process_iBUS(){
    //Read latest bytes available (includes flushing)
    uint8_t flag=0;
    uint8_t flushed=0;
    for(uint8_t i=0;i<n_max;i++){
        //Check if bytes available
        uint16_t bytes=uart.bytes_available();

        //Limit amount of bytes to be read
        if(bytes>buffer_size){
            bytes=buffer_size;
            flushed=1;
        }
        else flag=1;

        //If bytes available, read all of them
        if(bytes>0){        
            int16_t difference=(int16_t)index+(int16_t)bytes-(int16_t)buffer_size;
            if(difference>0)cut_buffer((uint16_t)difference);  
            uart.read_bytes((uint8_t*)(buffer+index),bytes);
            index+=bytes;
        }

        //Break if requested
        if(flag)break;
    }   

    //Flush (if needed)
    if(flushed){
        coms.send_msgs("RECEIVER: UART flushed...\n");
    }

    //Parse the messages
    uint8_t check_buffer=0;
    uint16_t i=0;
    for(uint8_t n=0;n<n_max;n++){
        //Search for a message
        check_buffer=0;
        for(i=0;i<index;i++){
            if(buffer[i]==0x20&&buffer[i+1]==0x40){
                check_buffer=1;
                break;
            }
        }
        
        //Process if message was found
        if(check_buffer){
            //Check if there is enough
            if(index-i>msg_size+2){    
                //Cut buffer
                cut_buffer(i+2);

                //Copy buffer to iBUS structure
                memcpy(&iBUS,buffer,msg_size);

                //if checksum was valid, reset failsafes
                if(validate_iBUS_checksum()){
                    reset_failsafes();
                    cut_buffer(msg_size);
                }
            }else cut_buffer(i);
        }
    }
}

void RECEIVER::set_flight_state(){
    //Check receiver is active
    if(!active)return;
    
    //Detect arming
    if(channels[enable_ch]>800&&!flight_state){
        state_change_counter++;
        if(state_change_counter>state_change_limit){
            state_change_counter=0;
            flight_state=1;
            sys.arm(RC);
        }
        return;
    }
    
    //Detect disarming
    if(channels[enable_ch]<200&&(flight_state||force_disarm)){
        state_change_counter++;
        if(state_change_counter>state_change_limit){
            state_change_counter=0;
            flight_state=0;
            sys.disarm(RC);
        }
        return;
    }
}

void RECEIVER::update(uint8_t flag){
    check_failsafes();
    process_iBUS();
    check_signals();
    offset_channels();
    apply_dead_zones();
    limit_outputs();
    set_flight_state();
    set_flight_mode();
    check_aux1();
    check_aux2();
    if(flag)print();
}

void RECEIVER::set_flight_mode(){
    //Check receiver is active
    if(!active)return;
    
    //Iterate through modes
    for(uint8_t i=0;i<n_modes;i++){
        //Detect Current Mode
        if(channels[mode_ch]<mode_limits[i]+10.0f){
            //Detect Mode Change Request
            if(flight_mode!=mode[i]){
                //Check that it is stable
                mode_change_counter++;
                if(mode_change_counter>mode_change_limit){
                    mode_change_counter=0;
                    flight_mode=mode[i];
                    control.set_mode(mode[i],RC);
                }
            }else mode_change_counter=0;
            return;
        }
    }
}

void RECEIVER::check_aux1(){
    //Check receiver is active
    if(!active)return;
    
    //Detect activation change
    if(channels[aux1_ch]>800&&!aux1_state){
        aux1_change_counter++;
        if(aux1_change_counter>aux1_change_limit){
            aux1_change_counter=0;
            aux1_state=1;
            control.set_home(&nav_ekf.xyz(0,0));
        }
        return;
    }
    
    //Detect deactivation change
    if(channels[aux1_ch]<200&&aux1_state){
        aux1_change_counter++;
        if(aux1_change_counter>aux1_change_limit){
            aux1_change_counter=0;
            aux1_state=0;
        }
        return;
    }
}

void RECEIVER::check_aux2(){
    //Check receiver is active
    if(!active)return;
    
    //Detect activation change
    if(channels[aux2_ch]>800&&!aux2_state){
        aux2_change_counter++;
        if(aux2_change_counter>aux2_change_limit){
            aux2_change_counter=0;
            aux2_state=1;
            control.set_mode(RTL,RC);
        }
        return;
    }

    //Detect deactivation change
    if(channels[aux2_ch]<200&&aux2_state){
        aux2_change_counter++;
        if(aux2_change_counter>aux2_change_limit){
            aux2_change_counter=0;
            aux2_state=0;
        }
        return;
    }
}

void RECEIVER::reset_channels(){
    for(uint8_t i=0;i<n_ch;i++)channels[i]=0;
}

void RECEIVER::check_signals(){
    uint8_t flag=0;
    uint16_t ch_eps=50;
    for(uint8_t i=0;i<active_ch;i++)if(raw_channels[i]>upper_limits[i]+ch_eps||raw_channels[i]<lower_limits[i]-ch_eps)flag=1;   

    //Check failsafes
    if(!flag){
        bounds_timer=sys.Time;
        bounds=0;
        return;
    }

    //Trigger failsafes
    float timer=sys.Time-bounds_timer;
    if(timer>bounds_timer_limit&&!bounds){
        bounds=1;    
        coms.send_msgs("RC Bounds!\n");
        sys.trigger_failsafe(bounds_fs);
        return;
    }
}


void RECEIVER::check_failsafes(){
    //Process failsafe (with timer)
    float period=sys.Time-timer;
    for(uint8_t i=0;i<n_fs;i++){
        if(period>fs_lim[i]&&!fs_triggered[i]){
            coms.send_msgs("RC Lost!\n");
            sprintf(msgs,"Time RC Lost %f \n",period);
            coms.send_msgs();
            reset_channels();
            fs_triggered[i]=1;
            active=0;
            sys.trigger_failsafe(fs[i]);
            return;
        }
    }
}

void RECEIVER::reset_failsafes(){
    for(uint8_t i=0;i<n_fs;i++)fs_triggered[i]=false;
    timer=sys.Time;
    if(!active){
        active=1;
        coms.send_msgs("RC Connected!\n");
    }
}

void RECEIVER::limit_outputs(){
    uint8_t flag=0;
    for(uint8_t i=0;i<active_ch;i++){
        if(channels[i]>ch_max[i]+dead_zones[i]){
            channels[i]=ch_max[i];
            flag=1;
        }
        if(channels[i]<ch_min[i]-dead_zones[i]){
            channels[i]=ch_min[i];
            flag=1;
        }
    }

        //Check failsafe
    float timer=sys.Time-bounds2_timer;
    if(!flag){
        bounds2_timer=sys.Time;
        bounds2=0;
    }
    
    //Trigger failsafe
    if(timer>bounds2_timer_limit&&!bounds2){
        bounds2=1;    
        coms.send_msgs("RC Bounds2!\n");
        sys.trigger_failsafe(bounds2_fs);
        return;
    }
}

void RECEIVER::apply_dead_zones(){
    for(uint8_t i=0;i<active_ch;i++){
        if(fabs(channels[i])<dead_zones[i])channels[i]=0;
    }
}


void RECEIVER::update_failsafes(int16_t buffer[]){
    memcpy(fs,buffer,n_fs*2);
    memcpy(&bounds_fs,buffer+n_fs,2);
    memcpy(&bounds2_fs,buffer+n_fs+1,2);
    for(uint8_t i=0;i<n_fs;i++)fs_lim[i]=(float)buffer[i+n_fs+2]/100.0f;
    bounds_timer_limit=(float)buffer[2*n_fs+2]/100.0f;
    bounds2_timer_limit=(float)buffer[2*n_fs+2+1]/100.0f;
    load_or_save_failsafes(1);
}

void RECEIVER::load_or_save_failsafes(uint8_t save){
    uint8_t flag=0;
    if(!save){
        save_read_param_uint16(file_name,failsafes_index,(uint16_t*)fs,n_fs,0);
        save_read_param_float(file_name,failsafes_timers_index,fs_lim,n_fs,0);
        save_read_param_uint16(file_name,bounds_fs_index,(uint16_t*)&bounds_fs,1,0);
        save_read_param_float(file_name,bounds_fs_timers_index,&bounds_timer_limit,1,0);
        save_read_param_uint16(file_name,bounds2_fs_index,(uint16_t*)&bounds_fs,1,0);
        save_read_param_float(file_name,bounds2_fs_timers_index,&bounds2_timer_limit,1,0);
    }
    //missing failsafe checks
    if(flag||save){
        save_read_param_uint16(file_name,failsafes_index,(uint16_t*)fs,n_fs,1);
        save_read_param_float(file_name,failsafes_timers_index,fs_lim,n_fs,1);
        save_read_param_uint16(file_name,bounds_fs_index,(uint16_t*)&bounds_fs,1,1);
        save_read_param_float(file_name,bounds_fs_timers_index,&bounds_timer_limit,1,1);
        save_read_param_uint16(file_name,bounds2_fs_index,(uint16_t*)&bounds_fs,1,1);
        save_read_param_float(file_name,bounds2_fs_timers_index,&bounds2_timer_limit,1,1);
    }
    print_failsafes();
}

void RECEIVER::print_failsafes(){
    coms.send_msgs("RC Failsafes\n");
    for(uint8_t i=0;i<n_fs;i++){
        sprintf(msgs,"Failsafe %d: %s @ t = %.2f\n",i,failsafe_msgs[fs[i]],fs_lim[i]);
        coms.send_msgs();
    }
    
    sprintf(msgs,"RC Bounds Failsafe: %s @ t = %.2f\n",failsafe_msgs[bounds_fs],bounds_timer_limit);
    coms.send_msgs();
    
    sprintf(msgs,"RC Bounds2 Failsafe: %s @ t = %.2f\n",failsafe_msgs[bounds2_fs],bounds2_timer_limit);
    coms.send_msgs();
}