/**
 * @file demo_1_motor.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */
//The main function starts two threads, one for the control loop and one for the printing loop. 
/*The code uses the following libraries:

std::tuple: a template class that represents a fixed-size collection of heterogeneous values. In this code, it is used to define the MotorAndSlider type, which is a tuple that contains a shared pointer to a MotorInterface object and a shared pointer to an AnalogSensorInterface object    std::shared_ptr: a smart pointer that manages the lifetime of an object and ensures that it is destroyed when no longer needed. The code uses shared_ptr to manage objects of the CanBus, MotorBoard, Motor, and AnalogSensor classes.
blmc_drivers: a library that provides an interface to communicate with the motor board and its components.
real_time_tools: a library that provides tools for real-time programming.
*/

#include <tuple>

#include "blmc_drivers/devices/analog_sensor.hpp"
#include "blmc_drivers/devices/can_bus.hpp"
#include "blmc_drivers/devices/motor.hpp"
#include "blmc_drivers/devices/motor_board.hpp"

typedef std::tuple<std::shared_ptr<blmc_drivers::MotorInterface>,
                   std::shared_ptr<blmc_drivers::AnalogSensorInterface>>
    MotorAndSlider;

/*<!--
This code defines a struct named Hardware, which contains four member variables:

    can_bus: a shared pointer to an object of type blmc_drivers::CanBusInterface, which represents the interface to communicate with the CAN bus.
    motor_board: a shared pointer to an object of type blmc_drivers::MotorBoardInterface, which represents the interface to communicate with the motor board.
    motor: a shared pointer to an object of type blmc_drivers::MotorInterface, which represents the interface to control a motor.
    slider: a shared pointer to an object of type blmc_drivers::AnalogSensorInterface, which represents the interface to read a slider position.

This struct is used to organize the various hardware components required for controlling a motor using the BLMC (Brushless Motor Control) library.
-->*/
  
struct Hardware
{
    std::shared_ptr<blmc_drivers::CanBusInterface> can_bus;
    std::shared_ptr<blmc_drivers::MotorBoardInterface> motor_board;
    std::shared_ptr<blmc_drivers::MotorInterface> motor;
    std::shared_ptr<blmc_drivers::AnalogSensorInterface> slider;
};

/*<!--static is a keyword in C++ that specifies that the function or variable should have internal linkage, meaning that it can only be accessed within the current translation unit.-->

THREAD_FUNCTION_RETURN_TYPE is a macro that is typically defined to specify the return type of a real-time thread function. It can be a void or an integer, depending on the specific real-time library being used.

control_loop is the name of the function being defined.

void *hardware_ptr is a pointer to void that serves as a generic input argument for the function. The function expects a pointer to a structure of type Hardware as input, but since we can't define the input argument type as Hardware, we define it as a pointer to void instead.

The static_cast<Hardware *>(hardware_ptr) expression casts the pointer to void back to a pointer to the Hardware structure type. The * operator then dereferences the resulting pointer, giving us access to the Hardware object that was passed as input to the function. Finally, the &hardware expression creates a reference to the Hardware object so that we can access its member variables inside the function.
-->*/
static THREAD_FUNCTION_RETURN_TYPE control_loop(void *hardware_ptr)
{
    // cast input arguments to the right format --------------------------------
    Hardware &hardware = *(static_cast<Hardware *>(hardware_ptr));

/* In this line of code, hardware_ptr (which is a void* pointer) is first cast to a pointer of type Hardware* using a static_cast. This is done because hardware_ptr was cast to void* pointer when it was passed to control_loop. Casting it back to Hardware* allows the code to access the members of the Hardware struct.

The dereferencing operator * is then used to obtain the actual Hardware object that hardware_ptr points to. The resulting pointer is then assigned to a reference called hardware, which is a reference to the original Hardware object passed to the control_loop function.

By using a reference to the original Hardware object, the code can access and modify the members of the Hardware struct within the function, and the changes made to hardware will be reflected in the original object passed to control_loop.
*/
    // torque controller -------------------------------------------------------
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    while (true)
    {
        // We send the current to the motor
        hardware.motor->set_current_target(/*desired_current*/ 0.0);
        hardware.motor->send_if_input_changed();

        spinner.spin();
    }
}

static THREAD_FUNCTION_RETURN_TYPE printing_loop(void *hardware_ptr)
{
    // cast input arguments to the right format --------------------------------
    Hardware &hardware = *(static_cast<Hardware *>(hardware_ptr));

    // print info 
  
  /*In this code, hardware.can_bus->get_output_frame() is a method call to get the output frame of the CanBusInterface object stored in hardware.can_bus. The "->" is a shorthand for calling a method or accessing a member of a pointer or a pointer-like object.

The method get_output_frame() returns a pointer to a real_time_tools::RealTimeContainer object that contains the CAN frames that are being sent or received.

newest_timeindex() is a method of the RealTimeContainer class that returns the index of the newest element in the container. In other words, timeindex will hold the index of the latest CAN frame that was received or sent by the CanBusInterface object.

Overall, this code initializes timeindex to the index of the newest CAN frame in the output frame of the CanBusInterface object. This value will be used later in the printing_loop function to retrieve and print the contents of the most recent CAN frames.--------------------------------------------------------------
*/
    long int timeindex =
        hardware.can_bus->get_output_frame()->newest_timeindex();

    while (true)
    {
        long int received_timeindex = timeindex;
        // this will return the element with the index received_timeindex,
        // if this element does not exist anymore, it will return the oldest
        // element it still has and change received_timeindex to the appropriate
        // index.
        blmc_drivers::CanBusFrame can_frame =
            (*hardware.can_bus->get_output_frame())[received_timeindex];
        timeindex++;

        rt_printf("timeindex: %ld\n", timeindex);
        can_frame.print();
    }
    return THREAD_FUNCTION_RETURN_VALUE;
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "expected 2 arguments: can bus and motor index"
                  << std::endl;
        exit(-1);
    }

    std::string can_bus_name(argv[1]);
    int motor_index = std::atoi(argv[2]);

    std::cout << "can_bus_name: " << can_bus_name
              << " motor_index: " << motor_index << std::endl;

    Hardware hardware;
    // First of all one need to initialize the communication with the can bus.
    hardware.can_bus = std::make_shared<blmc_drivers::CanBus>(can_bus_name);

    // Then we create a motor board object that will use the can bus in order
    // communicate between this application and the actual motor board.
    // Important: the blmc motors are alinged during this stage.
    hardware.motor_board =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(hardware.can_bus);

    // create the motor object that have an index that define the port on which
    // they are plugged on the motor board. This object takes also a MotorBoard
    // object to be able to get the sensors and send the control consistantly.
    // These safe motors have the ability to bound the current that is given
    // as input.
    hardware.motor = std::make_shared<blmc_drivers::SafeMotor>(
        hardware.motor_board, motor_index);

    // start real-time control loop --------------------------------------------
    real_time_tools::RealTimeThread control_thread;
    control_thread.create_realtime_thread(&control_loop, &hardware);

    // start real-time printing loop -------------------------------------------
    real_time_tools::RealTimeThread printing_thread;
    printing_thread.create_realtime_thread(&printing_loop, &hardware);

    rt_printf("control loop started \n");
    control_thread.join();
    printing_thread.join();
    return 0;
}
