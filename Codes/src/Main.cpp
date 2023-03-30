#include <Definitions.h>
DATALOG datalog;
COMS coms;
CONTROL control;
SYSTEM sys;
MODEL model;
RECEIVER receiver;
BATTERY battery;
NAV_EKF nav_ekf;
SENSORS sensors;
INPUTS inputs;
int main(int argc, char *argv[]){
    //Initialisation
    printf("Oscar is amazing!\n");
    coms.begin(0);                              //Start coms
    inputs.begin(400);
    datalog.begin(100);                         //Start logging
    receiver.begin(5);                          //Start receiver
    control.begin(5);                           //Start controllers
    sensors.begin(5);                           //Start sensors
    nav_ekf.begin();                            //Start NAV EKF
    battery.begin();                            //Start battery
    model.begin(5);                             //Start model
    sys.begin(5);                               //Start system
    
    //Loop
    while(true){
        sys.update();
        sensors.update(0);
        if(sys.armed){
            control.update(0);
        }else{
            inputs.disarm();
            control.reset();
            datalog.close_bin();
            model.reset();
        }
        receiver.update(0);
        coms.receive_data();
        coms.send_data();
        datalog.write_bin();
        sensors.update_extra(0);
        model.simulate(0);
        battery.update();
        sys.run_checks(0);
        sleep(1);
        // if(!execute)sleep(1);                       //Do NOT remove this sleep cause you will lock the CPU and lose all connectivity "ssh,etc..."
        // else execute=0;
    }
    
    //Power off
    return 0;
}
