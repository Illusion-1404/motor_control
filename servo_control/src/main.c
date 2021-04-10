/*************************** INCLUDE HEADER FILES *****************************/
# include <zephyr.h>
# include <device.h>
# include <devicetree.h>

// Include necessary headers from the Zephyr Headers
# include <drivers/gpio.h>   // Header Files to control the GPIO pins
# include <drivers/pwm.h>   // Header file to use pwm

// C Headers
# include <stdio.h>

// ROS Headers
# include <rcl/rcl.h>
# include <rclc/executor.h>   // Header file to spwan communication in threads
# include "servo_control_msg/msg/servo_control.h"  // Custom ROS Message
/******************************************************************************/

// Check if ROS Libraries have been imported properly
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

/**************************** GLOBAL SERVO VARIABLES **************************/
# define PERIOD (USEC_PER_SEC / 50U)  /* period of servo motor signal -> 20ms (50Hz)
// ( Definition from Zephyr Documentation )

/* all in micro second */
#define STEP 100    /* PWM pulse step */
#define MINPULSEWIDTH 1000  /* Servo 0 degrees */
#define MIDDLEPULSEWIDTH 1500   /* Servo 90 degrees */
#define MAXPULSEWIDTH 2000  /* Servo 180 degrees */
/******************************************************************************/

// Global ROS variables
rcl_publisher_t motor_state;  // Publishes the current position of servo motor
rcl_subscription_t motor_control;   // Subscribes to command to change servo position

servo_control_msg__msg__ServoControl servo1; // Create a message file to control servo 1

// Main Function
void main(void)
{
    // Define the required variables
    uint8_t dir = 0U;

    printf("Servo On and Off program \n");

    // Set initial position to 0 degrees
    uint32_t pulsewidth = MINPULSEWIDTH; 

}