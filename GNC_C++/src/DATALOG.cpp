#include <Definitions.h>



char device[] = "/dev/mmcblk0p1";

int DATALOG::is_mounted (char * dev_path) {
    FILE * mtab = NULL;
    struct mntent * part = NULL;
    int is_mounted_num = 0;
    if ( ( mtab = setmntent ("/etc/mtab", "r") ) != NULL) {
        while ( ( part = getmntent ( mtab) ) != NULL) {
            if ( ( part->mnt_fsname != NULL ) 
            && ( strcmp ( part->mnt_fsname, dev_path ) ) == 0 ) {
                is_mounted_num = 1;
            }
        }
        endmntent ( mtab);
    }
    return is_mounted_num;
}

void DATALOG::begin(uint8_t new_limit){
    coms.send_msgs("Initializing Data Logging...\n");
    limit=new_limit;
    while(!mounted){
        check_mount();
        if(!mounted)if(system("sudo mount -t vfat /dev/mmcblk0p1 /media/SD"));
        rc_usleep(500000);
    }
    open_bin();
    coms.send_msgs("Data Logging Initialized!\n");
    sleep(1);
}

void DATALOG::write_bin(){
    open_bin();
    if(bin_file_open&&!disable_logging){        
        //Reset index
        index=0;

        //Log/Fill data array
        log(&sys.Time,1);
        log(&nav_ekf.xyz(0,0),3);
        log(&nav_ekf.xyz_vel(0,0),3);
        log(&nav_ekf.xyz_acc(0,0),3);
        log(&nav_ekf.quat(0,0),4);
        log(&nav_ekf.rpy(0,0),3);
        log(&nav_ekf.pqr(0,0),3);
        log(&nav_ekf.xyz_meas(0,0),3);
        log(&nav_ekf.xyz_vel_meas(0,0),3);
        log(&nav_ekf.rpy_meas(0,0),3);
        log(&nav_ekf.acc(0,0),3);
        log(control.xyz_refs,3);
        log(control.xyz_vel_refs,3);
        log(control.rpy_refs,3);
        log(control.rate_refs,3);
        log(control.LMNZ,4);
        log(receiver.channels,8);
        
        //Write
        fseek(my_file,iter*LOG_BYTES,SEEK_SET);
        fwrite(data,1,LOG_BYTES,my_file);
        fflush(my_file);
        iter++;
    }
}

void DATALOG::log(float buffer[],uint16_t size){
    if(index+size<LOG_SIZE){
        memcpy(data+index,buffer,size*4);
        index+=size;
    }
}

void DATALOG::reset_counter(){
    counter=0;
}

void DATALOG::open_bin(){
    char file_name[50]="/media/SD/Flight_Data.bin";
    if(!bin_file_open&&mounted){
        if(!bin_file_created)my_file=fopen(file_name,"wb");
        else my_file=fopen(file_name,"ab+");
        if(my_file!=NULL){
            bin_file_open=1;
            bin_file_created=1;
        }
    }
}

void DATALOG::close_bin(){
    if(counter<limit)counter++;
    else{
        counter=0;
        if(!mounted){
            check_mount();
        }else{
            if(bin_file_open){
                if(!fclose(my_file))bin_file_open=0;
            }
        }
    }
}

void DATALOG::check_mount(){
    if (is_mounted(device)) {
        mounted=1;
        backup_logs("/media/SD/Flight_Data",LOG_BACKUPS,1);
        backup_logs("/media/SD/Sys_Log",LOG_BACKUPS,0);
        backup_parameters(LOG_BACKUPS);
        coms.send_msgs("Datalog Mounted!\n");
    }else{
        mounted=0;
        coms.send_msgs("Datalog Not Mounted!\n");
    }
}

void DATALOG::backup_logs(const char base_file[40],uint8_t n,uint8_t type){
    char bin_type[6]=".bin";
    char txt_type[6]=".txt";
    char file_type[6];
    if(type)memcpy(file_type,bin_type,6);
    else memcpy(file_type,txt_type,6);
    char cmd[100];
    for(uint8_t i=n;i>0;i--){
        //Copy using system command
        if(i>1)sprintf(cmd,"cp %s%d%s %s%d%s",base_file,i-1,file_type,base_file,i,file_type);
        else sprintf(cmd,"cp %s%s %s%d%s",base_file,file_type,base_file,i,file_type);
        if(system(cmd));
    }
}

void DATALOG::backup_parameters(uint8_t n){
    char cmd[100];
    for(uint8_t i=n;i>0;i--){
        //Copy using system command
        if(i>1)sprintf(cmd,"cp -r /media/SD/Param%d /media/SD/Param%d",i-1,i);
        else sprintf(cmd,"cp -r /media/SD/Param /media/SD/Param%d",i);
        if(system(cmd));
    }
    sprintf(cmd,"cp -r ./Param /media/SD");
    if(system(cmd));
}