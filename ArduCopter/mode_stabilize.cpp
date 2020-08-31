#include "Copter.h"

extern Fhan_Data ADRCROLL;
extern Fhan_Data ADRCPITCH;
extern Fhan_Data ADRCYAW;
extern Fhan_Data ADRC_ESO_autotune;

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
    raise_time_flag = true ; 
    disturbance_raise_time_flag = true ;
    temp_z1 = 0;

    ADRC_ESO_autotune.b0 = attitude_control->_adrc_t_b0; 

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
         
    
    if (roll_dis_radio_in > 1700 && ADRC_ESO_autotune.b0 < 200.0) // ch13 bigger than 1700 then start call disturbance
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
                attitude_control->rate_bf_pitch_target(4500.0);  

            }
            
            // time is small than 1s calculate error 1
            if ( millis() < autotune_start_time + 1000.0) //now = 0 - 1
            {   
                // calculate error1
                fitness_function_1(ADRCPITCH.target_velocity);
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
        /*
        if (millis() < autotune_start_time + 4000.0)
        {
            // step 2 calulate error 
            //error of z1 and real vel
            ADRC_ESO_autotune.ADRC_ESO_error1 += fabsf( ADRC_ESO_autotune.z1 - ADRCPITCH.actual_velocity ) ;
            //error of z2 and real dis
            ADRC_ESO_autotune.ADRC_ESO_error2 += fabsf( (-ADRC_ESO_autotune.z2/ADRC_ESO_autotune.b0) - attitude_control->pitch_disturbance ) ;
        }
        */

        // over 3 second reset flag and update ADRC.parameter
        if (millis() > autotune_start_time + 3000.0)        // now > 3.0 
        {   
            reset_ADRC_test(); // reset test ESO

            // step 3 update b0 and set msg to gcs
            attitude_control->_adrc_t_b0 += 10.0 ;  
            gcs().send_text(MAV_SEVERITY_INFO, "b0:%f  w0:%f", ADRC_ESO_autotune.b0,ADRC_ESO_autotune.w0);

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
    ADRC_ESO_autotune.z1 = 0.0;
    ADRC_ESO_autotune.z2 = 0.0;
    ADRC_ESO_autotune.ADRC_ESO_error1 = 0.0;
    ADRC_ESO_autotune.ADRC_ESO_error2 = 0.0;
    temp_z1 = 0.0;
    temp_z2 = 0.0;
    raise_time_flag = true ; 
    disturbance_raise_time_flag = true ;
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
void Copter::ModeStabilize::fitness_function_1(float target_velocity_temp)
{
    // fitness function for z1, use actual velocity
    // we need calculate raise time, overshoot, steady state error, peak time

    if ( ADRC_ESO_autotune.z1 > 0.9 * target_velocity_temp)
    {   
        // calculate raise time
        if (raise_time_flag)
        {
            raise_time =  millis() - autotune_start_time; // TODO need to be logged
            raise_time_flag = false ;
        }
        total_steady_state_error += fabsf( ADRC_ESO_autotune.z1 - ADRCPITCH.actual_velocity );   
    }

    // calculate max z1 to calculate overshoot
    if (ADRC_ESO_autotune.z1 > temp_z1)
    {
        temp_z1 = ADRC_ESO_autotune.z1;   
        peak_time = millis() - autotune_start_time;     
    }    

}

// fitness function 2 for z2
void Copter::ModeStabilize::fitness_function_2(float actual_disturbance_temp)
{
    float ESO_disturbance = ( -( ADRC_ESO_autotune.z2 / ADRC_ESO_autotune.b0 ) );
    // fitness function for z1, use actual velocity
    // we need calculate raise time, overshoot, steady state error, peak time

    if ( ESO_disturbance > 0.9 * actual_disturbance_temp)
    {   
        // calculate raise time
        if (disturbance_raise_time_flag)
        {
            disturbance_raise_time =  millis() - dis_start_time; // TODO need to be logged
            disturbance_raise_time_flag = false ;
        }
        total_disturbance_steady_state_error += fabsf( ESO_disturbance - actual_disturbance_temp );   
    }

    // calculate max disturbance to calculate overshoot
    if (ESO_disturbance > temp_z2)
    {
        temp_z2 = ESO_disturbance;   
        disturbance_peak_time = millis() - dis_start_time;     
    }    

}

// calculate final result of z1 and z2
void Copter::ModeStabilize::record_final_result(float target_velocity_temp, float actual_disturbance_temp)
{
    // record final error
    // z1 : steady state error , raise time , peak time , overshoot
    ADRC_ESO_autotune.ADRC_ESO_z1_error = total_steady_state_error + raise_time + peak_time + ( (temp_z1-target_velocity_temp) / target_velocity_temp ) ;
    ADRC_ESO_autotune.ADRC_ESO_z2_error = total_disturbance_steady_state_error + disturbance_raise_time + disturbance_peak_time + ( (temp_z2-actual_disturbance_temp) / actual_disturbance_temp ) ;

}
