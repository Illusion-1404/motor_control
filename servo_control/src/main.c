/*************************** INCLUDE HEADER FILES *****************************/
# include <zephyr.h>
# include <device.h>
# include <devicetree.h>
#include <sys/printk.h>   // More general printk function

// Include necessary headers from the Zephyr Headers
# include <drivers/gpio.h>   // Header Files to control the GPIO pins
# include <drivers/pwm.h>   // Header file to use pwm

// ROS Headers
# include <rcl/rcl.h>
# include <rclc/rclc.h>
# include <rclc/executor.h>   // Header file to spwan communication in threads
# include "servo_control_msg/msg/servo_control.h"  // Custom ROS Message
/******************************************************************************/

// Check if ROS Libraries have been imported properly
#define RCCHECK(fn) { 
    rcl_ret_t temp_rc = fn; 
    if((temp_rc != RCL_RET_OK)){
        printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); 
        return 1;
        }
    }

#define RCSOFTCHECK(fn) { 
    rcl_ret_t temp_rc = fn; 
    if((temp_rc != RCL_RET_OK)){
        printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);
        }
    }

/**************************** GLOBAL SERVO VARIABLES **************************/
# define PERIOD (USEC_PER_SEC / 50U)  /* period of servo motor signal -> 20ms (50Hz)
// ( Definition from Zephyr Documentation )

/* all in micro second */
#define SERVO_STEP 100    /* PWM pulse step */
#define MINPULSEWIDTH 1000  /* Servo 0 degrees */
#define MIDDLEPULSEWIDTH 1500   /* Servo 90 degrees */
#define MAXPULSEWIDTH 2000  /* Servo 180 degrees */

#define SLEEP_TIME_S 1 /* Sleep Time in s */
/* **************************************************************************** */

// Global ROS variables
rcl_publisher_t motor_state_publisher;  // Publishes the current position of servo motor
rcl_subscription_t motor_state_subsciber;   // Subscribes to command to change servo position
const char* ROS_TOPIC_NAME = "/ServoMotor";  // ROS Topic Name

servo_control_msg__msg__ServoControl servo1_pub_msg; // Publish message file
servo_control_msg__msg__ServoControl servo1_sub_msg; // Subscribe message file

static struct device *nucleo_board; // Create a device object for servo motor

// Define the required pins as macros
#define PWM_PIN DT_NODE_LABEL(pwm12)
#define PWM_LED_PIN DT_ALIAS(pwm-led0) 

void PWM_control(bool* dir, uint32_t* pulse_width)
{
    if(*dir)
    {
        if(*pulse_width < MAXPULSEWIDTH)  *pulse_width += SERVO_STEP;
        else *dir = false;
    }
    else
    {
        if(*pulse_width > MINPULSEWIDTH)  *pulse_width -= SERVO_STEP;
        else *dir = true;
    }

}

void get_degrees_from_pwm(uint32_t* degrees, uint32_t* pulse_width)
{
    *degrees = (uint32_t)(float((*pulse_width - MINPULSEWIDTH)/(MAXPULSEWIDTH - MINPULSEWIDTH)*180.0));
}

void timer_callback(rclc_timer_t * timer)
{
    rclc_ret_t rc;
    if(timer != NULL)
    {
        rc = rcl_publish(&motor_state_publisher, &servo1_pub_msg, NULL);
        if(rc == RCL_RET_OK)  printk("Published Message \n");
        else printk("Message not published \n");
    }
    else printk("Error in timer callback function \n");
}

void subscriber_callback(const void* incomingMsg)
{
    const servo_control_msg__msg__ServoControl* message = (const 
                                    servo_control_msg__msg__ServoControl *) incomingMsg; 
    printk("Received Servo State: %d, PWM value: %d, Motor Direction: %d", 
                            message->servo_state, message->pulse_width, message->direction);
}

// Main Function
void main(void)
{
    printk("Servo Motor Control Program using microROS \n");

    servo1 = device_get_binding(PWM_PIN);

    // Define the required variables    
    uint32_t pulse_width = MINPULSEWIDTH; // Set initial position to 0 degrees    
    bool dir = true;   // Initially the motor is set to move forward
    uint32_t degrees = 0; // Set initial position to 0 degrees 
    bool servo_state = true; // Initially servo is Off

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_support_t support;
    rcl_ret_t rc;

    // create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "servo_control", "", &support));

    // Creating the Motor State Subscriber
	RCCHECK(rclc_subscription_init_default(&motor_state_subsciber, &node, 
            ROSIDL_GET_MSG_TYPE_SUPPORT(servo_control_msg, msg, ServoControl), ROS_TOPIC_NAME));
    servo_control_msg__msg__ServoControl__init(&servo1_sub_msg);
    

    // Create the Motor State Publisher
    RCCCHECK(rclc_publisher_init_default(&motor_state_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(servo_control_msg, msg, ServoControl), ROS_TOPIC_NAME));
    servo_control_msg__msg__ServoControl__init(&servo1_pub_msg);
    
    // Set the values to be published to the ROS
    servo1_pub_msg.servo_state = servo_state;
    servo1_pub_msg.pulse_width = pulse_width;
    servo1_pub_msg.direction = dir;
    servo1_pub_msg.degrees = degrees;

    rcl_timer_t publish_timer;
    const unsigned int timer_timeout = 1000; //in ms
    RCCHECK(rclc_timer_init_default(&publish_timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));


    // Creating a executor
	rclc_executor_t executor;
    // #handles = #Subscriptions + #Timers
    unsigned int num_handles = 1 + 1;
    printk("Number of handles = %u", num_handles);
	RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
	
    // Add Subscriber to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_state_subscriber, &servo1_sub_msg, 
            &subscriber_callback, ON_NEW_DATA));
    

    // Add Timer to executor
    RCCCHECK(rclc_executor_add_timer(&executor, &publish_timer));

    while(1)
    {
        // Check if PWM Pin is accessible
        if(pwm_pin_set_usec(servo1, PWM_PIN, PERIOD, pulse_width, 0) < 0)
        {
            printk("PWM assigning has failed \n");
            return;
        }
        
        // Timeout for the executor
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        get_degrees_from_pwm(&degrees, &pulse_width);

		// printk("PWM pulse width: %d\n", pulse_width);
		// printk("Degrees: %d\n", degrees);
                
        PWM_control(&dir, &pulse_width);

		k_sleep(K_SECONDS(SLEEP_TIME_S));
    }
}