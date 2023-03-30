#include <Definitions.h>

char PID_msg[]="PID SETUP";

void PID::begin(const char msg[]){
    //Copy name
    strcpy(name,msg);
    
    //Load Parameters
    load_parameters();
}

void PID::begin(const char msg[],float parameters[]){
    //Copy name
    strcpy(name,msg);
    
    //Save Parameters
    save_parameters(parameters);
}

void PID::load_parameters(){
    float parameters[10];
    save_read_param_float(name,0,parameters,10,0);
    update_parameters(parameters);
}

void PID::save_parameters(float parameters[]){
    if(update_parameters(parameters))save_read_param_float(name,0,parameters,10,1);
}

uint8_t PID::update_parameters(float parameters[]){
    //Check parameters not empty
    float norm=0;
    for(uint8_t i=0;i<10;i++)norm+=fabs(parameters[i]);
    if(norm<eps){
        coms.send_msgs("PID Error: Empty Parameter Set!\n");
        return false;
    }

    //Write parameters
    memcpy(&p,parameters,40);
    
    //Check parameters
    if(check_parameters()){
        init=1;
        print_parameters();
        return true;
    }
    else {
        init=0;
        coms.send_msgs("PID Error: Parameters!\n");
        return false;
    }
}

float PID::update(float yk,float rk, float delta_t, uint8_t integrate,uint8_t kick){
    float uk;
    if(init){
        //Check time stamp to reset controller
        uint8_t flag=0;
        float delta_t_up;
        if(sys.Time-time_stamp>2*delta_t){
            reset();
            flag=1;
        }else{
            delta_t_up = sys.Time-time_stamp;
        }
        time_stamp=sys.Time;

        //Update error
        if(!flag)rk=p.alpha_r*r1+(1-p.alpha_r)*rk;              //Filter for reference
        // float ek_t=rk-yk;                                   //Temporal variable for error
        float ek_t=rk-yk;
        if(ek_t>p.max_ek)ek_t=p.max_ek;                         //Limit error to max
        if(ek_t<-p.max_ek)ek_t=-p.max_ek;                       //Limit error to -max
        ek=p.alpha_e*ek1+(1-p.alpha_e)*ek_t;                    //Filter for error
        
        //Update derivative
        if(!flag){
            float ek_dot_t;                                     //Temporal variable for error_derivative
            // if(kick)ek_dot_t=(ek-ek1)/delta_t;   
            if(kick)ek_dot_t=(ek-ek1)/delta_t_up;                      
            else ek_dot_t=-(yk-y1)/delta_t_up;
            if(ek_dot_t>p.max_ek_dot)ek_dot_t=p.max_ek_dot;         //Limit error derivative to max
            if(ek_dot_t<-p.max_ek_dot)ek_dot_t=-p.max_ek_dot;       //Limit error derivative to -max
            ek_dot=p.alpha_d*ek_dot1+(1-p.alpha_d)*ek_dot_t;        //Filter for derivative
        }else ek_dot=0;
        
        //Update integral
        if(integrate==1)ei_k=ei_k+delta_t_up*ek;                    //Full Error Integration
        if(integrate==2)ei_k=ei_k+p.alpha_i*delta_t_up*ek;            //Smooth Error Integration
        
        //Saturate integral state (Anti-windup)
        float I=ei_k*p.Ki;
        if(fabs(I)>p.Imax){
            if(ei_k>0){
                ei_k=p.Imax/p.Ki;
                I=p.Imax;
            }else{
                ei_k=-p.Imax/p.Ki;
                I=-p.Imax;
            }
        }
        
        //Move data
        ek1=ek;
        ek_dot1=ek_dot;
        r1=rk;
        y1=yk;

        //Calculate output
        uk=p.Kp*ek+p.Kd*ek_dot+I;
    }else{
        //Uninitialised (shut-down)
        reset();
        uk=0;
    }
    return uk;
}

float PID::update_xyz(float yk,float rk, float delta_t, uint8_t integrate,uint8_t kick, float y_dot){
    float uk;
    if(init){
        //Check time stamp to reset controller
        uint8_t flag=0;
        float delta_t_up;
        if(sys.Time-time_stamp>2*delta_t){
            reset();
            flag=1;
        }else{
            delta_t_up = sys.Time-time_stamp;
        }
        time_stamp=sys.Time;

        //Update error
        if(!flag)rk=p.alpha_r*r1+(1-p.alpha_r)*rk;              //Filter for reference
        // float ek_t=rk-yk;                                   //Temporal variable for error
        float ek_t=rk-yk;
        if(ek_t>p.max_ek)ek_t=p.max_ek;                         //Limit error to max
        if(ek_t<-p.max_ek)ek_t=-p.max_ek;                       //Limit error to -max
        ek=p.alpha_e*ek1+(1-p.alpha_e)*ek_t;                    //Filter for error
        
        //Update derivative
        if(!flag){
            float ek_dot_t;                                     //Temporal variable for error_derivative
            // if(kick)ek_dot_t=(ek-ek1)/delta_t;   
            if(kick)ek_dot_t=(ek-ek1)/delta_t_up;                      
            else ek_dot_t=-y_dot;
            if(ek_dot_t>p.max_ek_dot)ek_dot_t=p.max_ek_dot;         //Limit error derivative to max
            if(ek_dot_t<-p.max_ek_dot)ek_dot_t=-p.max_ek_dot;       //Limit error derivative to -max
            ek_dot=p.alpha_d*ek_dot1+(1-p.alpha_d)*ek_dot_t;        //Filter for derivative
        }else ek_dot=0;
        
        //Update integral
        if(integrate==1)ei_k=ei_k+delta_t_up*ek;                    //Full Error Integration
        if(integrate==2)ei_k=ei_k+p.alpha_i*delta_t_up*ek;            //Smooth Error Integration
        
        //Saturate integral state (Anti-windup)
        float I=ei_k*p.Ki;
        if(fabs(I)>p.Imax){
            if(ei_k>0){
                ei_k=p.Imax/p.Ki;
                I=p.Imax;
            }else{
                ei_k=-p.Imax/p.Ki;
                I=-p.Imax;
            }
        }
        
        //Move data
        ek1=ek;
        ek_dot1=ek_dot;
        r1=rk;
        y1=yk;

        //Calculate output
        uk=p.Kp*ek+p.Kd*ek_dot+I;
    }else{
        //Uninitialised (shut-down)
        reset();
        uk=0;
    }
    return uk;
}

float PID::update_yaw(float yk,float rk, float delta_t, uint8_t integrate,uint8_t kick){
    float uk;
    if(init){
        //Check time stamp to reset controller
        uint8_t flag=0;
        float delta_t_up;
        if(sys.Time-time_stamp>2*delta_t){
            reset();
            flag=1;
        }else{
            delta_t_up = sys.Time-time_stamp;
        }
        time_stamp=sys.Time;

        //Update error
        if(!flag)rk=p.alpha_r*r1+(1-p.alpha_r)*rk;              //Filter for reference
        // float ek_t=rk-yk;                                   //Temporal variable for error
        float ek_t=rk-yk;

        if(ek_t>180)ek_t-=360;                         //Limit error to max
        if(ek_t<-180)ek_t+=360;

        if(ek_t>p.max_ek)ek_t=p.max_ek;                         //Limit error to max
        if(ek_t<-p.max_ek)ek_t=-p.max_ek;                       //Limit error to -max
        ek=p.alpha_e*ek1+(1-p.alpha_e)*ek_t;                    //Filter for error
        
        //Update derivative
        if(!flag){
            float ek_dot_t;                                     //Temporal variable for error_derivative
            // if(kick)ek_dot_t=(ek-ek1)/delta_t;   
            if(kick)ek_dot_t=(ek-ek1)/delta_t_up;                      
            else ek_dot_t=-(yk-y1)/delta_t_up;
            if(ek_dot_t>p.max_ek_dot)ek_dot_t=p.max_ek_dot;         //Limit error derivative to max
            if(ek_dot_t<-p.max_ek_dot)ek_dot_t=-p.max_ek_dot;       //Limit error derivative to -max
            ek_dot=p.alpha_d*ek_dot1+(1-p.alpha_d)*ek_dot_t;        //Filter for derivative
        }else ek_dot=0;
        
        //Update integral
        if(integrate==1)ei_k=ei_k+delta_t_up*ek;                    //Full Error Integration
        if(integrate==2)ei_k=ei_k+p.alpha_i*delta_t_up*ek;            //Smooth Error Integration
        
        //Saturate integral state (Anti-windup)
        float I=ei_k*p.Ki;
        if(fabs(I)>p.Imax){
            if(ei_k>0){
                ei_k=p.Imax/p.Ki;
                I=p.Imax;
            }else{
                ei_k=-p.Imax/p.Ki;
                I=-p.Imax;
            }
        }
        
        //Move data
        ek1=ek;
        ek_dot1=ek_dot;
        r1=rk;
        y1=yk;

        //Calculate output
        uk=p.Kp*ek+p.Kd*ek_dot+I;
    }else{
        //Uninitialised (shut-down)
        reset();
        uk=0;
    }
    return uk;
}

void PID::reset(){
    ei_k=0;
    ek1=0;
    ek_dot1=0;
    r1=0;
    y1=0;
}

uint8_t PID::check_parameters(){    
    if(p.Kp<0.0001||p.Kp>1000)return false;
    if(p.Kd<0||p.Kd>100)return false;
    if(p.Ki<0.0001||p.Ki>100)return false;
    if(p.Imax<0||p.Imax>1000)return false;
    if(p.alpha_e<0||p.alpha_e>1)return false;
    if(p.alpha_d<0||p.alpha_d>1)return false;
    if(p.alpha_r<0||p.alpha_r>1)return false;
    if(p.alpha_i<0||p.alpha_i>1)return false;
    if(p.max_ek<0||p.max_ek>1000)return false;
    if(p.max_ek_dot<0||p.max_ek_dot>1000)return false;
    return true;
}

void PID::print_parameters(){
    sprintf(msgs,"%s %s\n",name,PID_msg);
    coms.send_msgs();    
    sprintf(msgs,"Kp=%.2f\n",p.Kp);
    coms.send_msgs();
    sprintf(msgs,"Ki=%.2f\n",p.Ki);
    coms.send_msgs();
    sprintf(msgs,"Kd=%.2f\n",p.Kd);
    coms.send_msgs();
    sprintf(msgs,"Imax=%.2f\n",p.Imax);
    coms.send_msgs();
    sprintf(msgs,"alpha_d=%.2f\n",p.alpha_d);
    coms.send_msgs();
    sprintf(msgs,"alpha_r=%.2f\n",p.alpha_r);
    coms.send_msgs();
    sprintf(msgs,"alpha_e=%.2f\n",p.alpha_e);
    coms.send_msgs();
    sprintf(msgs,"alpha_i=%.2f\n",p.alpha_i);
    coms.send_msgs();
    sprintf(msgs,"max_ek=%.2f\n",p.max_ek);
    coms.send_msgs();
    sprintf(msgs,"max_ek_dot=%.2f\n",p.max_ek_dot);
    coms.send_msgs();
}