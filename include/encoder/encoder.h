/**
 * @file encoder.h
 * @author Noa Sendlhofer
 */

/*
This code is to read and debounce a KY-040 rotary encoder (maybe others) using interrupts on the Raspbery Pi PICO.
This code should also work on other processors with a few changes to the hardware specific parts of the code.

Here is a crude represntation of the output of the encoder

CW rotation
______       ______
      F1____|			Phase A
_________       ______
	F2_____|		Phase B
Notice that Phase A falling edge (F1 in the diagram) occurs before Phase B (F2) for CW rotation



CCW Rotation
_________      ______
	F2____|			Phase A
______       ______
      F1____|			Phase B
Notice that Phase B falling (F1 in the diagram) edge occurs before Phase A (F2)for CCW rotation

This code works by watching for the first falling edge (F1) and setting cw_fall TRUE if Phase A is the first falling edge
and setting ccw_fall TRUE is phase B is the first falling edge. After one of these (cw_fall or ccw_fall) have been set
the code watches to see which phase is next to trigger the interrupt (F2).  After the second (F2) interrupt, the code can detrmine the
direction that the encoder has been turned.

cw  A leads B
ccw B leads A
this may change depending on your wiring and what encoder is used


The code does not handle the encoder push button switch.
This can be implemented by adding additional code to hangle the switch interrupt
       if (gpio == ENC_SW)
		{
			//handle switch event here
		}

*/

/**BEGIN CODE HERE****************************************************************************************************/


#ifndef PICO_MOTORS_ENCODER_H_
#define PICO_MOTORS_ENCODER_H_

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"

#include "pio_rotary_encoder.pio.h"

class BaseEncoder
{
public:
    // set the current rotation to a specific value
    virtual void set_rotation(int _rotation) = 0;

    // get the current rotation
    virtual int get_rotation(void) = 0;
};

// class to read the rotation of the rotary encoder
class RotaryEncoder : public BaseEncoder
{
public:
    // constructor
    // rotary_encoder_A is the pin for the A of the rotary encoder.
    // The B of the rotary encoder has to be connected to the next GPIO.
    RotaryEncoder(uint8_t rotary_encoder_A, uint8_t rotary_encoder_B);

    // set the current rotation to a specific value
    void set_rotation(int _rotation) override
    {
        rotation = _rotation;
    }

    // get the current rotation
    int get_rotation(void) override
    {
        return rotation;
    }

private:
    static void pio_irq_handler()
    {
        // test if irq 0 was raised
        if (pio0_hw->irq & 1)
        {
            rotation = rotation - 1;
        }
        // test if irq 1 was raised
        if (pio0_hw->irq & 2)
        {
            rotation = rotation + 1;
        }
        // clear both interrupts
        pio0_hw->irq = 3;
    }

    // the pio instance
    PIO pio;
    // the state machine
    uint sm;
    // the current location of rotation
    static int rotation;
};

class RotaryEncoder1 : public BaseEncoder
{
public:
    // constructor
    // rotary_encoder_A is the pin for the A of the rotary encoder.
    // The B of the rotary encoder has to be connected to the next GPIO.
    RotaryEncoder1(uint8_t rotary_encoder_A, uint8_t rotary_encoder_B);

    // set the current rotation to a specific value
    void set_rotation(int _rotation) override
    {
        rotation = _rotation;
    }

    // get the current rotation
    int get_rotation(void) override
    {
        return rotation;
    }

private:
    static void pio_irq_handler()
    {
        // test if irq 0 was raised
        if (pio1_hw->irq & 1)
        {
            rotation = rotation - 1;
        }
        // test if irq 1 was raised
        if (pio1_hw->irq & 2)
        {
            rotation = rotation + 1;
        }
        // clear both interrupts
        pio1_hw->irq = 3;
    }

    // the pio instance
    PIO pio;
    // the state machine
    uint sm;
    // the current location of rotation
    static int rotation;
};

#endif //PICO_MOTORS_ENCODER_H_
