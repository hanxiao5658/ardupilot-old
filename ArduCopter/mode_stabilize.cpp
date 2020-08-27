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
    time_record_flag = true;

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

    //diable those 2 functions so that we can override rate_control    
    //if (roll_dis_radio_in < 1700 )
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
    // we have 3 steps : 
    //                      1.  call disturbance , 
    //                      2.  calculate error between ESO.z2/b0 and disturbance
    //                      3.  update b0
         
    
    if (roll_dis_radio_in > 1700 && ADRC_ESO_autotune.b0 < 200.0) // ch13 bigger than 1700 then start call disturbance
    {   
                  
        // step 1 call disturbance , disturbance_ch(13) is control channel for tunning
        // disturbance should last 1 seconds , and 1 second for no disturbance .
        // every tunning loop last for 2 seconds (1 second disturbance + 1 second no disturbance)
        
        if (time_record_flag)
        {
            // reset test ESO
            ADRC_ESO_autotune.z1 = 0.0;
            ADRC_ESO_autotune.z2 = 0.0;
            ADRC_ESO_autotune.ADRC_ESO_error1 = 0.0;
            ADRC_ESO_autotune.ADRC_ESO_error2 = 0.0;

            //attitude_control->rate_bf_pitch_target(1500.0);  // over ride pitch rate control
            attitude_control->pitch_disturbance_flag = 1.0;  // disturbance_on 
            dis_start_time = millis();                      // record disturbance start time
            time_record_flag = false;                      // dis record time  
        }
        
        // check timeout 
        if (millis() > dis_start_time + 2000.0) 
        {
            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, aparm.angle_max);
            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            // timeout end disturbance 
            attitude_control->pitch_disturbance_flag = 0.0;
        }
        
        if (millis() < dis_start_time + 4000.0)
        {
            // step 2 calulate error 
            //error of z1 and real vel
            ADRC_ESO_autotune.ADRC_ESO_error1 += fabs( ADRC_ESO_autotune.z1 - ADRCPITCH.actual_velocity ) ;
            //error of z2 and real dis
            ADRC_ESO_autotune.ADRC_ESO_error2 += fabs( (-ADRC_ESO_autotune.z2/ADRC_ESO_autotune.b0) - attitude_control->pitch_disturbance ) ;
        }

        if (millis() > dis_start_time + 4000.0)
        {   
            // over 2 second start a new tunning loop

            // reset time record flag
            time_record_flag = true; 

            // record final error
            ADRC_ESO_autotune.ADRC_ESO_z1_error = ADRC_ESO_autotune.ADRC_ESO_error1;
            ADRC_ESO_autotune.ADRC_ESO_z2_error = ADRC_ESO_autotune.ADRC_ESO_error2;

            
            // step 3 update b0
            attitude_control->_adrc_t_b0 += 10.0 ;  
            gcs().send_text(MAV_SEVERITY_INFO, "b0:%f  w0:%f", ADRC_ESO_autotune.b0,ADRC_ESO_autotune.w0);

        }
          
        
    } 
    else
    {
        attitude_control->pitch_disturbance_flag = 0.0;  // disturbance off

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, aparm.angle_max);
   
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    }
    
    

}
