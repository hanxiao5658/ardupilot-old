#include "Copter.h"


extern Fhan_Data ADRCROLL;
extern Fhan_Data ADRCPITCH;
extern Fhan_Data ADRCYAW;
extern Fhan_Data ADRC_ESO_autotune1;
extern Fhan_Data ADRC_ESO_autotune2;
extern Fhan_Data ADRC_ESO_autotune3;
extern Fhan_Data ADRC_ESO_autotune4;
extern Fhan_Data ADRC_ESO_autotune5;

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::ModeStabilize::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    
    ///////////////////////////////
    // use this to decide record time 
    time_record_flag_1 = true;
    time_record_flag_2 = true;
    override_body_rate_flage = false;
    ADRC_ESO_autotune1.raise_time_flag = true ; 
    ADRC_ESO_autotune1.disturbance_raise_time_flag = true ;
    ADRC_ESO_autotune1.max_ESO_z1 = 0;
    ADRC_ESO_autotune1.b0 = attitude_control->_adrc_t_b0; 


    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::ModeStabilize::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // get radio of tunning ch13
    uint16_t roll_dis_radio_in =  RC_Channels::rc_channel(attitude_control->tun_ch - 1)->get_radio_in();

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    AP_Vehicle::MultiCopter &aparm = copter.aparm;

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    //disable those 2 functions so that we can override rate_control    
    if ( !override_body_rate_flage )
    {
        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, aparm.angle_max);
   
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    }
 
    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // every autotune circle has 3 seconds
    // 1st second : override body rate control, call a 90 deg/s rate control to test z1
    // 2nd second : call disturbance , test z2
    // 2*0.5 second : return to level
         
    
    if (roll_dis_radio_in > 1700 && ADRC_ESO_autotune1.b0 < 2000.0) // ch13 bigger than 1700 then start call disturbance
    {   
                  
        // step 1 : 1st second : override body rate control, call a 90 deg/s rate control to test z1
        {
            if (time_record_flag_1)                                 //now = 0
            {   
                // disable record time 
                time_record_flag_1 = false;  

                autotune_start_time = millis(); //autotune start time  

                // override body rate control to call a 90 deg/s rate control
                override_body_rate_flage = true;

                // call a 90 deg/s rate control 
                attitude_control->rate_bf_pitch_target(-3000.0);  

            }
            
            // time is small than 1s calculate error 1
            if ( millis() < autotune_start_time + 1000.0) //now = 0 - 1
            {   
                // calculate error1
                fitness_function_1();
                target_velocity = ADRCPITCH.target_velocity;
            }

            // if step 1 is over 1s                                 now = 1 - 1.5
            if ( millis() > autotune_start_time + 1000.0) 
            {
                // let go body rate control,return to level
                override_body_rate_flage = false;
                        
            }
        }
        // step 2 call disturbance                                  now = 1.5 - 2.5
        // 0.5s to make sure we are level
        {
            if ( autotune_start_time + 2500.0> millis() && millis()> autotune_start_time + 1500.0) 
            {
                // start disturbance 
                disturbance_switch( true ); 
                actual_disturbance = attitude_control->pitch_disturbance;
                // record disturbance start time
                if (time_record_flag_2)
                {
                    time_record_flag_2 = false;
                    // record disturbance start time
                    dis_start_time = millis();   
                }

                // calculate error2
                fitness_function_2(attitude_control->pitch_disturbance);                
            }

            // end disturbance                              now = 2.5 - 3.0
            if (autotune_start_time + 3000.0 > millis() && millis()> autotune_start_time + 2500.0) 
            {
                // end disturbance 
                disturbance_switch( false );      

                // record final error
                record_final_result(target_velocity, actual_disturbance);           
            }
        }

        // over 3 second reset flag and update ADRC.parameter
        if (millis() > autotune_start_time + 3000.0)        // now > 3.0 
        {   
            reset_ADRC_test(); // reset test ESO
            update_parameter(); // update parameters 
            gcs().send_text(MAV_SEVERITY_INFO, "b0:%f  w0:%f", ADRC_ESO_autotune1.b0,ADRC_ESO_autotune1.w0);

        }
          
        
    } 
    else
    {
        // switch is low or b0 is too big
        disturbance_switch( false );        // turn off disturbance
        override_body_rate_flage = false;   // NOT override rate control
        
    }
    
    

}

// reset ADRC
void Copter::ModeStabilize::reset_ADRC_test()
{
        // reset test ESO
    {
        ADRC_ESO_autotune1.z1 = 0.0;
        ADRC_ESO_autotune1.z2 = 0.0;
        ADRC_ESO_autotune1.ADRC_ESO_error1 = 0.0;
        ADRC_ESO_autotune1.ADRC_ESO_error2 = 0.0;
        ADRC_ESO_autotune1.total_disturbance_steady_state_error = 0;
        ADRC_ESO_autotune1.total_steady_state_error = 0;
        ADRC_ESO_autotune1.max_ESO_z1 = 0.0;
        ADRC_ESO_autotune1.max_ESO_z2 = 0.0;
        ADRC_ESO_autotune1.raise_time_flag = true ; 
        ADRC_ESO_autotune1.disturbance_raise_time_flag = true ;

        ADRC_ESO_autotune2.z1 = 0.0;
        ADRC_ESO_autotune2.z2 = 0.0;
        ADRC_ESO_autotune2.ADRC_ESO_error1 = 0.0;
        ADRC_ESO_autotune2.ADRC_ESO_error2 = 0.0;
        ADRC_ESO_autotune2.total_disturbance_steady_state_error = 0;
        ADRC_ESO_autotune2.total_steady_state_error = 0;
        ADRC_ESO_autotune2.max_ESO_z1 = 0.0;
        ADRC_ESO_autotune2.max_ESO_z2 = 0.0;
        ADRC_ESO_autotune2.raise_time_flag = true ; 
        ADRC_ESO_autotune2.disturbance_raise_time_flag = true ;

        ADRC_ESO_autotune3.z1 = 0.0;
        ADRC_ESO_autotune3.z2 = 0.0;
        ADRC_ESO_autotune3.ADRC_ESO_error1 = 0.0;
        ADRC_ESO_autotune3.ADRC_ESO_error2 = 0.0;
        ADRC_ESO_autotune3.total_disturbance_steady_state_error = 0;
        ADRC_ESO_autotune3.total_steady_state_error = 0;
        ADRC_ESO_autotune3.max_ESO_z1 = 0.0;
        ADRC_ESO_autotune3.max_ESO_z2 = 0.0;
        ADRC_ESO_autotune3.raise_time_flag = true ; 
        ADRC_ESO_autotune3.disturbance_raise_time_flag = true ;

        ADRC_ESO_autotune4.z1 = 0.0;
        ADRC_ESO_autotune4.z2 = 0.0;
        ADRC_ESO_autotune4.ADRC_ESO_error1 = 0.0;
        ADRC_ESO_autotune4.ADRC_ESO_error2 = 0.0;
        ADRC_ESO_autotune4.total_disturbance_steady_state_error = 0;
        ADRC_ESO_autotune4.total_steady_state_error = 0;
        ADRC_ESO_autotune4.max_ESO_z1 = 0.0;
        ADRC_ESO_autotune4.max_ESO_z2 = 0.0;
        ADRC_ESO_autotune4.raise_time_flag = true ; 
        ADRC_ESO_autotune4.disturbance_raise_time_flag = true ;

        ADRC_ESO_autotune5.z1 = 0.0;
        ADRC_ESO_autotune5.z2 = 0.0;
        ADRC_ESO_autotune5.ADRC_ESO_error1 = 0.0;
        ADRC_ESO_autotune5.ADRC_ESO_error2 = 0.0;
        ADRC_ESO_autotune5.total_disturbance_steady_state_error = 0;
        ADRC_ESO_autotune5.total_steady_state_error = 0;
        ADRC_ESO_autotune5.max_ESO_z1 = 0.0;
        ADRC_ESO_autotune5.max_ESO_z2 = 0.0;
        ADRC_ESO_autotune5.raise_time_flag = true ; 
        ADRC_ESO_autotune5.disturbance_raise_time_flag = true ;
    }
    // reset time record flag
    time_record_flag_1 = true; 
    time_record_flag_2 = true;
}

// turn on/off distrurbance
void Copter::ModeStabilize::disturbance_switch(bool flag)
{
    if (flag)
    {
       attitude_control->pitch_disturbance_flag = 1.0; //turn on the disturbance
    }
    else
    {
        attitude_control->pitch_disturbance_flag = 0.0; // turn off disturbance
    }
}

// fitness function 1 for z1
void Copter::ModeStabilize::fitness_function_1()
{
    // fitness function for z1, use actual velocity
    // we need calculate raise time, overshoot, steady state error, peak time

    //test1
    {
        if ( ADRC_ESO_autotune1.z1 > 0.9 * ADRCPITCH.actual_velocity)
        {   
            // calculate raise time
            if (ADRC_ESO_autotune1.raise_time_flag)
            {
                ADRC_ESO_autotune1.raise_time =  millis() - autotune_start_time; // TODO need to be logged
                ADRC_ESO_autotune1.raise_time_flag = false ;
            }
            
        }

        ADRC_ESO_autotune1.total_steady_state_error += fabsf( ADRC_ESO_autotune1.z1 - ADRCPITCH.actual_velocity ); 

        // calculate max z1 to calculate overshoot
        if (ADRC_ESO_autotune1.z1 > ADRC_ESO_autotune1.max_ESO_z1)
        {
            ADRC_ESO_autotune1.max_ESO_z1 = ADRC_ESO_autotune1.z1;  

            // when ESO.z1 get max record real vel to calculat overshot
            real_vel = ADRCPITCH.actual_velocity;

            // cal overshoot
            ADRC_ESO_autotune1.overshoot_z1 = fabsf(ADRC_ESO_autotune1.max_ESO_z1 - real_vel) / fabsf(real_vel);
            ADRC_ESO_autotune1.peak_time = millis() - autotune_start_time;     
        }  

          
    }

        //test2
    {
        if ( ADRC_ESO_autotune2.z1 > 0.9 * ADRCPITCH.actual_velocity)
        {   
            // calculate raise time
            if (ADRC_ESO_autotune2.raise_time_flag)
            {
                ADRC_ESO_autotune2.raise_time =  millis() - autotune_start_time; // TODO need to be logged
                ADRC_ESO_autotune2.raise_time_flag = false ;
            }
            
        }

        ADRC_ESO_autotune2.total_steady_state_error += fabsf( ADRC_ESO_autotune2.z1 - ADRCPITCH.actual_velocity ); 

        // calculate max z1 to calculate overshoot
        if (ADRC_ESO_autotune2.z1 > ADRC_ESO_autotune2.max_ESO_z1)
        {
            ADRC_ESO_autotune2.max_ESO_z1 = ADRC_ESO_autotune2.z1;  

            // when ESO.z1 get max record real vel to calculat overshot
            real_vel = ADRCPITCH.actual_velocity;

            // cal overshoot
            ADRC_ESO_autotune2.overshoot_z1 = fabsf(ADRC_ESO_autotune2.max_ESO_z1 - real_vel) / fabsf(real_vel);
            ADRC_ESO_autotune2.peak_time = millis() - autotune_start_time;     
        }    
    }

        //test3
    {
        if ( ADRC_ESO_autotune3.z1 > 0.9 * ADRCPITCH.actual_velocity)
        {   
            // calculate raise time
            if (ADRC_ESO_autotune3.raise_time_flag)
            {
                ADRC_ESO_autotune3.raise_time =  millis() - autotune_start_time; // TODO need to be logged
                ADRC_ESO_autotune3.raise_time_flag = false ;
            }
            
        }

        ADRC_ESO_autotune3.total_steady_state_error += fabsf( ADRC_ESO_autotune3.z1 - ADRCPITCH.actual_velocity ); 

        // calculate max z1 to calculate overshoot
        if (ADRC_ESO_autotune3.z1 > ADRC_ESO_autotune1.max_ESO_z1)
        {
            ADRC_ESO_autotune3.max_ESO_z1 = ADRC_ESO_autotune3.z1;  

            // when ESO.z1 get max record real vel to calculat overshot
            real_vel = ADRCPITCH.actual_velocity;

            // cal overshoot
            ADRC_ESO_autotune3.overshoot_z1 = fabsf(ADRC_ESO_autotune3.max_ESO_z1 - real_vel) / fabsf(real_vel);
            ADRC_ESO_autotune3.peak_time = millis() - autotune_start_time;     
        }    
    }

        //test4
    {
        if ( ADRC_ESO_autotune4.z1 > 0.9 * ADRCPITCH.actual_velocity)
        {   
            // calculate raise time
            if (ADRC_ESO_autotune4.raise_time_flag)
            {
                ADRC_ESO_autotune4.raise_time =  millis() - autotune_start_time; // TODO need to be logged
                ADRC_ESO_autotune4.raise_time_flag = false ;
            }
            
        }

        ADRC_ESO_autotune4.total_steady_state_error += fabsf( ADRC_ESO_autotune4.z1 - ADRCPITCH.actual_velocity ); 

        // calculate max z1 to calculate overshoot
        if (ADRC_ESO_autotune4.z1 > ADRC_ESO_autotune1.max_ESO_z1)
        {
            ADRC_ESO_autotune4.max_ESO_z1 = ADRC_ESO_autotune4.z1;  

            // when ESO.z1 get max record real vel to calculat overshot
            real_vel = ADRCPITCH.actual_velocity;

            // cal overshoot
            ADRC_ESO_autotune4.overshoot_z1 = fabsf(ADRC_ESO_autotune4.max_ESO_z1 - real_vel) / fabsf(real_vel);
            ADRC_ESO_autotune4.peak_time = millis() - autotune_start_time;     
        }    
    }

        //test5
    {
        if ( ADRC_ESO_autotune5.z1 > 0.9 * ADRCPITCH.actual_velocity)
        {   
            // calculate raise time
            if (ADRC_ESO_autotune5.raise_time_flag)
            {
                ADRC_ESO_autotune5.raise_time =  millis() - autotune_start_time; // TODO need to be logged
                ADRC_ESO_autotune5.raise_time_flag = false ;
            }
            
        }

        ADRC_ESO_autotune5.total_steady_state_error += fabsf( ADRC_ESO_autotune5.z1 - ADRCPITCH.actual_velocity ); 

        // calculate max z1 to calculate overshoot
        if (ADRC_ESO_autotune5.z1 > ADRC_ESO_autotune1.max_ESO_z1)
        {
            ADRC_ESO_autotune5.max_ESO_z1 = ADRC_ESO_autotune5.z1;  

            // when ESO.z1 get max record real vel to calculat overshot
            real_vel = ADRCPITCH.actual_velocity;

            // cal overshoot
            ADRC_ESO_autotune5.overshoot_z1 = fabsf(ADRC_ESO_autotune5.max_ESO_z1 - real_vel) / fabsf(real_vel);
            ADRC_ESO_autotune5.peak_time = millis() - autotune_start_time;     
        }    
    }

}

// fitness function 2 for z2
void Copter::ModeStabilize::fitness_function_2(float actual_disturbance_temp)
{
    float ESO_disturbance = 0.0 ;
    // fitness function for z1, use actual velocity
    // we need calculate raise time, overshoot, steady state error, peak time
    
    // test1
    {
        ESO_disturbance = ( -( ADRC_ESO_autotune1.z2 / ADRC_ESO_autotune1.b0 ) );

        if ( ESO_disturbance > 0.9 * actual_disturbance_temp)
        {   
            // calculate raise time
            if (ADRC_ESO_autotune1.disturbance_raise_time_flag)
            {
                ADRC_ESO_autotune1.disturbance_raise_time =  millis() - dis_start_time; // TODO need to be logged
                ADRC_ESO_autotune1.disturbance_raise_time_flag = false ;
            }
            
        }

        ADRC_ESO_autotune1.total_disturbance_steady_state_error += fabsf( ESO_disturbance - actual_disturbance_temp ); 
        
        // calculate max disturbance to calculate overshoot
        if (ESO_disturbance > ADRC_ESO_autotune1.max_ESO_z2)
        {
            ADRC_ESO_autotune1.max_ESO_z2 = ESO_disturbance;   
            ADRC_ESO_autotune1.overshoot_z2 = fabsf(ADRC_ESO_autotune1.max_ESO_z2 - actual_disturbance_temp) / fabsf(actual_disturbance_temp);
            ADRC_ESO_autotune1.disturbance_peak_time = millis() - dis_start_time;     
        }    
    }

    // test2
    {
        ESO_disturbance = ( -( ADRC_ESO_autotune2.z2 / ADRC_ESO_autotune2.b0 ) );

        if ( ESO_disturbance > 0.9 * actual_disturbance_temp)
        {   
            // calculate raise time
            if (ADRC_ESO_autotune2.disturbance_raise_time_flag)
            {
                ADRC_ESO_autotune2.disturbance_raise_time =  millis() - dis_start_time; // TODO need to be logged
                ADRC_ESO_autotune2.disturbance_raise_time_flag = false ;
            }
            
        }

        ADRC_ESO_autotune2.total_disturbance_steady_state_error += fabsf( ESO_disturbance - actual_disturbance_temp ); 
        
        // calculate max disturbance to calculate overshoot
        if (ESO_disturbance > ADRC_ESO_autotune2.max_ESO_z2)
        {
            ADRC_ESO_autotune2.max_ESO_z2 = ESO_disturbance;   
            ADRC_ESO_autotune2.overshoot_z2 = fabsf(ADRC_ESO_autotune2.max_ESO_z2 - actual_disturbance_temp) / fabsf(actual_disturbance_temp);
            ADRC_ESO_autotune2.disturbance_peak_time = millis() - dis_start_time;     
        }    
    }

    // test3
    {
        ESO_disturbance = ( -( ADRC_ESO_autotune3.z2 / ADRC_ESO_autotune3.b0 ) );

        if ( ESO_disturbance > 0.9 * actual_disturbance_temp)
        {   
            // calculate raise time
            if (ADRC_ESO_autotune3.disturbance_raise_time_flag)
            {
                ADRC_ESO_autotune3.disturbance_raise_time =  millis() - dis_start_time; // TODO need to be logged
                ADRC_ESO_autotune3.disturbance_raise_time_flag = false ;
            }
            
        }

        ADRC_ESO_autotune3.total_disturbance_steady_state_error += fabsf( ESO_disturbance - actual_disturbance_temp ); 
        
        // calculate max disturbance to calculate overshoot
        if (ESO_disturbance > ADRC_ESO_autotune3.max_ESO_z2)
        {
            ADRC_ESO_autotune3.max_ESO_z2 = ESO_disturbance;   
            ADRC_ESO_autotune3.overshoot_z2 = fabsf(ADRC_ESO_autotune3.max_ESO_z2 - actual_disturbance_temp) / fabsf(actual_disturbance_temp);
            ADRC_ESO_autotune3.disturbance_peak_time = millis() - dis_start_time;     
        }    
    }

    // test4
    {
        ESO_disturbance = ( -( ADRC_ESO_autotune4.z2 / ADRC_ESO_autotune4.b0 ) );

        if ( ESO_disturbance > 0.9 * actual_disturbance_temp)
        {   
            // calculate raise time
            if (ADRC_ESO_autotune4.disturbance_raise_time_flag)
            {
                ADRC_ESO_autotune4.disturbance_raise_time =  millis() - dis_start_time; // TODO need to be logged
                ADRC_ESO_autotune4.disturbance_raise_time_flag = false ;
            }
            
        }

        ADRC_ESO_autotune4.total_disturbance_steady_state_error += fabsf( ESO_disturbance - actual_disturbance_temp ); 
        
        // calculate max disturbance to calculate overshoot
        if (ESO_disturbance > ADRC_ESO_autotune4.max_ESO_z2)
        {
            ADRC_ESO_autotune4.max_ESO_z2 = ESO_disturbance;  
            ADRC_ESO_autotune4.overshoot_z2 = fabsf(ADRC_ESO_autotune4.max_ESO_z2 - actual_disturbance_temp) / fabsf(actual_disturbance_temp); 
            ADRC_ESO_autotune4.disturbance_peak_time = millis() - dis_start_time;     
        }    
    }

    // test5
    {
        ESO_disturbance = ( -( ADRC_ESO_autotune5.z2 / ADRC_ESO_autotune5.b0 ) );

        if ( ESO_disturbance > 0.9 * actual_disturbance_temp)
        {   
            // calculate raise time
            if (ADRC_ESO_autotune5.disturbance_raise_time_flag)
            {
                ADRC_ESO_autotune5.disturbance_raise_time =  millis() - dis_start_time; // TODO need to be logged
                ADRC_ESO_autotune5.disturbance_raise_time_flag = false ;
            }
            
        }

        ADRC_ESO_autotune5.total_disturbance_steady_state_error += fabsf( ESO_disturbance - actual_disturbance_temp ); 
        
        // calculate max disturbance to calculate overshoot
        if (ESO_disturbance > ADRC_ESO_autotune5.max_ESO_z2)
        {
            ADRC_ESO_autotune5.max_ESO_z2 = ESO_disturbance;   
            ADRC_ESO_autotune5.overshoot_z2 = fabsf(ADRC_ESO_autotune5.max_ESO_z2 - actual_disturbance_temp) / fabsf(actual_disturbance_temp); 
            ADRC_ESO_autotune5.disturbance_peak_time = millis() - dis_start_time;     
        }    
    }


}

// calculate final result of z1 and z2
void Copter::ModeStabilize::record_final_result(float target_velocity_temp, float actual_disturbance_temp)
{
    // record final error
    // z1 : steady state error , raise time , peak time , overshoot

    // test 1
    ADRC_ESO_autotune1.ADRC_ESO_z1_error = ADRC_ESO_autotune1.total_steady_state_error + ADRC_ESO_autotune1.raise_time + ADRC_ESO_autotune1.peak_time + ADRC_ESO_autotune1.overshoot_z1 ;
    ADRC_ESO_autotune1.ADRC_ESO_z2_error = ADRC_ESO_autotune1.total_disturbance_steady_state_error + ADRC_ESO_autotune1.disturbance_raise_time + ADRC_ESO_autotune1.disturbance_peak_time + ADRC_ESO_autotune1.overshoot_z2 ;
    ADRC_ESO_autotune1.total_error = ADRC_ESO_autotune1.ADRC_ESO_z1_error + ADRC_ESO_autotune1.ADRC_ESO_z2_error;



    // test 2
    ADRC_ESO_autotune2.ADRC_ESO_z1_error = ADRC_ESO_autotune2.total_steady_state_error + ADRC_ESO_autotune2.raise_time + ADRC_ESO_autotune2.peak_time + ADRC_ESO_autotune2.overshoot_z1 ;
    ADRC_ESO_autotune2.ADRC_ESO_z2_error = ADRC_ESO_autotune2.total_disturbance_steady_state_error + ADRC_ESO_autotune2.disturbance_raise_time + ADRC_ESO_autotune2.disturbance_peak_time + ADRC_ESO_autotune2.overshoot_z2 ;
    ADRC_ESO_autotune2.total_error = ADRC_ESO_autotune2.ADRC_ESO_z1_error + ADRC_ESO_autotune2.ADRC_ESO_z2_error;

    // test 3
    ADRC_ESO_autotune3.ADRC_ESO_z1_error = ADRC_ESO_autotune3.total_steady_state_error + ADRC_ESO_autotune3.raise_time + ADRC_ESO_autotune3.peak_time + ADRC_ESO_autotune3.overshoot_z1 ;
    ADRC_ESO_autotune3.ADRC_ESO_z2_error = ADRC_ESO_autotune3.total_disturbance_steady_state_error + ADRC_ESO_autotune3.disturbance_raise_time + ADRC_ESO_autotune3.disturbance_peak_time + ADRC_ESO_autotune3.overshoot_z2 ;
    ADRC_ESO_autotune3.total_error = ADRC_ESO_autotune3.ADRC_ESO_z1_error + ADRC_ESO_autotune3.ADRC_ESO_z2_error;
   
    // test 4
    ADRC_ESO_autotune4.ADRC_ESO_z1_error = ADRC_ESO_autotune4.total_steady_state_error + ADRC_ESO_autotune4.raise_time + ADRC_ESO_autotune4.peak_time + ADRC_ESO_autotune4.overshoot_z1 ;
    ADRC_ESO_autotune4.ADRC_ESO_z2_error = ADRC_ESO_autotune4.total_disturbance_steady_state_error + ADRC_ESO_autotune4.disturbance_raise_time + ADRC_ESO_autotune4.disturbance_peak_time + ADRC_ESO_autotune4.overshoot_z2 ;
    ADRC_ESO_autotune4.total_error = ADRC_ESO_autotune4.ADRC_ESO_z1_error + ADRC_ESO_autotune4.ADRC_ESO_z2_error;
   
    // test 5
    ADRC_ESO_autotune5.ADRC_ESO_z1_error = ADRC_ESO_autotune5.total_steady_state_error + ADRC_ESO_autotune5.raise_time + ADRC_ESO_autotune5.peak_time + ADRC_ESO_autotune5.overshoot_z1 ;
    ADRC_ESO_autotune5.ADRC_ESO_z2_error = ADRC_ESO_autotune5.total_disturbance_steady_state_error + ADRC_ESO_autotune5.disturbance_raise_time + ADRC_ESO_autotune5.disturbance_peak_time + ADRC_ESO_autotune5.overshoot_z2 ;
    ADRC_ESO_autotune5.total_error = ADRC_ESO_autotune5.ADRC_ESO_z1_error + ADRC_ESO_autotune5.ADRC_ESO_z2_error;




}

// update ADRC parameters 
void Copter::ModeStabilize::update_parameter()
{
    // step 3 update b0 and set msg to gcs
    if (attitude_control->_adrc_t_b0 < 200)
    {
        attitude_control->_adrc_t_b0 += 10.0 ; 
    }
    else
    {
        attitude_control->_adrc_t_b0 += 100.0 ;
    }
}