// #include <Definitions.h>
#include <INPUTS.h>
#include <COMS.h>
#include <PRU.h>
#include <Types.h>
#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>

extern COMS coms;

void INPUTS::begin(uint16_t freq){
    coms.send_msgs("Initializing Inputs...\n");
    int32_t mem_fd;
    uint32_t *iram;
    uint32_t *ctrl;
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
    if(mem_fd==-1)coms.send_msgs("Unable to open /dev/mem\n");
    else{
        pwm = (struct pwm*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_RAM_BASE);
        iram = (uint32_t*) mmap(0, 0x2000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_IRAM_BASE);
        ctrl = (uint32_t*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_PRUSS_CTRL_BASE);
        close(mem_fd);
        *ctrl = 0;
        std::memcpy(iram, PRUcode, sizeof(PRUcode));
        *ctrl |= 2;
    }
    if(number_motors+number_servos>max_inputs){
        coms.send_msgs("Inputs: max_inputs reached!");
        while(1);
    }
    if(check_repeated_ch()){
        coms.send_msgs("Inputs: repeated channels!\n");
        while(1);
    }
    set_freq(0xFFFFFFFF, freq);
    disarm();
    coms.send_msgs("Inputs Initalized!\n");
    sleep(1);
}

uint8_t INPUTS::check_repeated_ch(){
    for(uint8_t i=0;i<number_motors;i++){
        for(uint8_t j=0;j<number_motors;j++){
            if(motor_channels[i]==motor_channels[j]&&i!=j)return true;
        }
        for(uint8_t j=0;j<number_servos;j++){
            if(motor_channels[i]==servo_channels[j])return true;
        }
    }
    for(uint8_t i=0;i<number_servos;i++){
        for(uint8_t j=0;j<number_motors;j++){
            if(servo_channels[i]==motor_channels[j])return true;
        }
        for(uint8_t j=0;j<number_servos;j++){
            if(servo_channels[i]==servo_channels[j]&&i!=j)return true;
        }
    }
    return false;
}

void INPUTS::set_freq(uint32_t chmask, uint16_t freq_hz){
    uint8_t i;
    uint32_t tick = TICK_PER_S / freq_hz;
    for(i = 0; i < PWM_CHAN_COUNT; i++) {
        if(chmask & (1U << i)) {
            pwm->channel[i].time_t = tick;
        }
    }
}

void INPUTS::disarm(){
    disarm_motors();
    zero_servos();
}

void INPUTS::disarm_motors(){
    for(uint8_t i=0;i<number_motors;i++)motors[i]=0;
    write_motors();
    for(uint8_t i=0;i<number_motors;i++)enable_ch(motor_channels[i]);
}

void INPUTS::zero_servos(){
    for(uint8_t i=0;i<number_servos;i++)angles[i]=0;
    write_servos();
    for(uint8_t i=0;i<number_servos;i++)enable_ch(servo_channels[i]);
}

void INPUTS::write_ch(uint8_t ch, uint16_t period_us)
{
    if(ch < PWM_CHAN_COUNT) {
            pwm->channel[ch].time_high = TICK_PER_US * period_us;
  }
}
void INPUTS::enable_ch(uint8_t ch)
{
    if(ch < PWM_CHAN_COUNT) {
        pwm->channelenable |= 1U << ch;
    }
}

void INPUTS::write_motors(){
    uint16_t value;
    for(uint8_t i=0;i<number_motors;i++){
        //Limit
        if(motors[i]>(float)range_us_motors)motors[i]=(float)range_us_motors;
        if(motors[i]<0)motors[i]=0;
        
        //Write
        value=round(motors[i]);
        value+=min_us_motors;
        write_ch(motor_channels[i],value);
    }
}

void INPUTS::write_servos(){
    for(uint8_t i=0;i<number_servos;i++){
        //Limit
        if(angles[i]>(float)range_us_servos)angles[i]=(float)range_us_servos;
        if(angles[i]<0)angles[i]=0;
        
        //Reverse
        if(reverse_servos[i])angles[i]=range_us_servos-angles[i];
        
        //Write
        write_ch(servo_channels[i],(uint16_t)angles[i]+min_us_servos);
    }
}

void INPUTS::write(){
    write_motors();
    write_servos();
}

void INPUTS::calibrate_motors(){
    coms.send_msgs("Calibrating motors...\n");
    
    //Prompt user inputs
    char msg1[50]="Remove propellers!\n";
    char msg2[50]="Disconnect Extra drones on network!\n";
    char msg3[50]="Disable broadcasting!\n";
    coms.send_msgs("First Warning...\n");
    coms.send_msgs(msg1);
    coms.send_msgs(msg2);
    coms.send_msgs(msg3);
    while(coms.any_message_available())rc_usleep(20000);
    rc_usleep(500000);
    while(!coms.any_message_available())rc_usleep(20000);
    rc_usleep(500000);
    
    coms.send_msgs("Second Warning...\n");
    coms.send_msgs(msg1);
    coms.send_msgs(msg2);
    coms.send_msgs(msg3);
    while(coms.any_message_available())rc_usleep(20000);
    rc_usleep(500000);
    while(!coms.any_message_available())rc_usleep(20000);
    rc_usleep(500000);

    //Third warning
    coms.send_msgs("Last Warning...\n");
    coms.send_msgs(msg1);
    coms.send_msgs(msg2);
    coms.send_msgs(msg3);
    while(coms.any_message_available())rc_usleep(20000);
    rc_usleep(500000);
    while(!coms.any_message_available())rc_usleep(20000);
    rc_usleep(500000);

    //Set motors high
    for(uint8_t i=0;i<number_motors;i++){
        write_ch(motor_channels[i],max_us_motors);
        enable_ch(i);
    }
    
    //Prompt user inputs
    coms.send_msgs("Calibrating... Send input when ready!\n");
    while(!coms.any_message_available())rc_usleep(20000);
    
    //Disarm
    disarm_motors();
}