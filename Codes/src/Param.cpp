#include <stdio.h>
#include <Param.h>
#include <Types.h>

void save_read_param_float(char name[], uint16_t index, float parameters[],uint16_t num, uint8_t type){
    uint8_t flag=0;
    FILE *f;
    sprintf(msgs,"./Param/%s.bin",name);
    f=fopen(msgs,"rb+");
    if(f==NULL){
        f=fopen(msgs,"wb+");
        flag=1;
    }
    fseek(f,index,SEEK_SET);
    if(type)fwrite(parameters,num*4,1,f);
    else {
        if(!flag){
            if(fread(parameters,num*4,1,f));
        }
        else for(uint8_t i=0;i<num;i++)parameters[i]=0;
    }
    fclose(f);
}

void save_read_param_int16(char name[], uint16_t index, int16_t parameters[],uint16_t num, uint8_t type){
    uint8_t flag=0;
    FILE *f;
    sprintf(msgs,"./Param/%s.bin",name);
    f=fopen(msgs,"rb+");
    if(f==NULL){
        f=fopen(msgs,"wb+");
        flag=1;
    }
    fseek(f,index,SEEK_SET);
    if(type)fwrite(parameters,num*2,1,f);
    else {
        if(!flag){
            if(fread(parameters,num*2,1,f));
        }
        else for(uint8_t i=0;i<num;i++)parameters[i]=0;
    }
    fclose(f);
}

void save_read_param_uint16(char name[], uint16_t index, uint16_t parameters[],uint16_t num, uint8_t type){
    uint8_t flag=0;
    FILE *f;
    sprintf(msgs,"./Param/%s.bin",name);
    f=fopen(msgs,"rb+");
    if(f==NULL){
        f=fopen(msgs,"wb+");
        flag=1;
    }
    fseek(f,index,SEEK_SET);
    if(type)fwrite(parameters,num*2,1,f);
    else {
        if(!flag){
            if(fread(parameters,num*2,1,f));
        }
        else for(uint8_t i=0;i<num;i++)parameters[i]=0;
    }
    fclose(f);
}