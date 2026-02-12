#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#define NUM_MOTORS 12
#define PUBLISH_DATA_FREQ 100
#define PUBLISH_INFO_FREQ 1


#define VELOCITY_FILTER  0.01


#define QUATERNION_FILTER 0.5
#define ANG_VEL_FILTER    0.5
#define ACC_FILTER        0.5



/* Set the delay between fresh samples */
#define BNO080_SAMPLERATE_DELAY_MS 5
#define COV_MATRIX_SAMPLE 100


#define CONTACT_DETECTOR_ANALOG_THRESHOLD 400

# endif
