#include "encoders/orbis_brd10/br10_sp.h"

#include "app_maverick_steeringcontrols.h"
#include "app_maverick_gen.h"


void maverick_steering_controls(){
    chRegSetThreadName("MAVERICK_STEERING_CONTROLS");
    spi_init();
    // commands_printf("Microseconds per clock ticks: %");
    steering_controls_isRunning = true;

    // Define all variables to be used in the loop
    steering_cmd latest_cmd;

    serial_no_response enc_serial_no_response;
    speed_response enc_speed_response;
    temp_response enc_temp_response;
    detailed_status_response enc_detailed_status_response;
    gen_response enc_gen_response;

    double output = 0;

    while(!stop_now) {
        // Read most current command
        if (chMtxTryLock(&steering_cmd_mtx)){
            latest_cmd = latest_steering_cmd;
            chMtxUnlock(&steering_cmd_mtx);
        } else {
            // TODO: Implement a mutex skip counter to ensure latest command isn't getting skipped every time
        }

        // Get the current encoder value
        steering_cmd_response response;
        response.cmd = latest_cmd.cmd;
        switch(response.cmd){
            case GET_SERIAL_NO:;
                enc_serial_no_response = br10_getSerialNoResponse();
                response.current_position = enc_serial_no_response.gen_response.position;
                response.isError = enc_serial_no_response.gen_response.isError;
                response.isWarning = enc_serial_no_response.gen_response.isWarning;
                // response.serial_no = enc_response.serial_no;
                break;
            case GET_SPEED:;
                enc_speed_response = br10_getSpeedResponse();
                response.current_position = enc_speed_response.gen_response.position;
                response.isError = enc_speed_response.gen_response.isError;
                response.isWarning = enc_speed_response.gen_response.isWarning;
                response.speed = enc_speed_response.speed;
                break;
            case GET_TEMP:;
                enc_temp_response = br10_getTempResponse();
                response.current_position = enc_temp_response.gen_response.position;
                response.isError = enc_temp_response.gen_response.isError;
                response.isWarning = enc_temp_response.gen_response.isWarning;
                response.temperature = enc_temp_response.temp;
                break;
            case GET_DETAILED_STATUS:;
                enc_detailed_status_response = br10_getDetailedStatusResponse();
                response.current_position = enc_detailed_status_response.gen_response.position;
                response.isError = enc_detailed_status_response.gen_response.isError;
                response.isWarning = enc_detailed_status_response.gen_response.isWarning;
                response.detailed_status = enc_detailed_status_response.status;
                break;
            default:;
                enc_gen_response = br10_getGeneralResponse();
                response.current_position = enc_gen_response.position;
                response.isError = enc_gen_response.isError;
                response.isWarning = enc_gen_response.isWarning;
                break;
        }
        // TODO: Add CRC check
        
        // Calculate PID
        chMtxLock(&steering_pid_mtx);
        steering_pid_values latest_pid = steering_pid;
        chMtxUnlock(&steering_pid_mtx);

        output = calculate_pid(latest_pid, response.current_position, latest_cmd.cmd_position);
        // commands_printf("Output Duty: %f", output);
        
        // Set motor
        if (output > 0) {
            output = map(output, 0, MAX_STEERING_DUTY, MIN_STEERING_DUTY, MAX_STEERING_DUTY);
        } else if (output < 0) {
            output = map(output, 0, -MAX_STEERING_DUTY, -MIN_STEERING_DUTY, -MAX_STEERING_DUTY);
        }
        mc_interface_set_duty(output);
        
        // Return command response
        chMtxLock(&steering_cmd_response_mtx);
        latest_steering_cmd_response = response;
        chMtxUnlock(&steering_cmd_response_mtx);
        // TODO: Implement a mailbox system
        // int return_cntr = 0;
        // while (!chMtxTryLock(&steering_cmd_response_mtx)){
        //     if (return_cntr > 5){
        //         commands_printf("Maverick Steering Controls Thread: "
        //             "Error trying to unlock command response mutex!")
        //     }            
        //     return_cntr++;
        //     chThdSleepMilliseconds(1);
        // }

        // Thread sleep
        // TODO: replace with sleep until to better control cycle speed
        timeout_reset();
        chThdSleepMilliseconds(100);
    }
    spi_deinit();
    chMtxUnlockAll();
    steering_controls_isRunning = false;
}