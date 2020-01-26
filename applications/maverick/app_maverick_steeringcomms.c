
#include "app_maverick_gen.h"
#include "app_maverick_steeringcomms.h"

void maverick_steeringcomms( arg){
    (void)arg;
    chRegSetThreadName("MAVERICK_STEERING_COMMS");
    steering_comms_isRunning = true;
    steering_commands last_cmd = NONE;
    steering_cmd next_cmd;
    steering_cmd_response last_response;
    mc_configuration *mc_config = mc_interface_get_configuration();
    while (!stop_now){

        // Testing switching commands
        next_cmd.cmd = NONE;
        if (next_cmd.cmd_position == 90){
            next_cmd.cmd_position = 270;
        } else {
            next_cmd.cmd_position = 90;
        }
        // next_cmd.cmd_position = steering_pid.angle_division;
        chMtxLock(&steering_cmd_mtx);
        latest_steering_cmd = next_cmd;
        chMtxUnlock(&steering_cmd_mtx);

        chMtxLock(&steering_cmd_response_mtx);
        last_response = latest_steering_cmd_response;
        commands_printf("Position: %f", last_response.current_position);
        commands_printf("Setpoint: %f", next_cmd.cmd_position);
        chMtxUnlock(&steering_cmd_response_mtx);

        last_cmd = next_cmd.cmd;
        
        // Update PID values if needed
        if (    mc_config->p_pid_kp != steering_pid.kp ||
                mc_config->p_pid_ki != steering_pid.ki ||
                mc_config->p_pid_kd != steering_pid.kd ||
                mc_config->p_pid_kd_filter != steering_pid.kd_filter ||
                mc_config->p_pid_ang_div != steering_pid.angle_division){
            chMtxLock(&steering_pid_mtx);
            steering_pid.kp = mc_config->p_pid_kp;
            steering_pid.ki = mc_config->p_pid_ki;
            steering_pid.kd = mc_config->p_pid_kd;
            steering_pid.kd_filter = mc_config->p_pid_kd_filter;
            steering_pid.angle_division = mc_config->p_pid_ang_div;
            chMtxUnlock(&steering_pid_mtx);
        }


        timeout_reset();
        chThdSleepMilliseconds(5000);
    }
    chMtxUnlockAll();
    steering_comms_isRunning = false;
}