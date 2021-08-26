/***********************************************************************
 * Hello Chirp! - an example application for ultrasonic sensing 
 *
 * This project is designed to be your first introduction to using
 * Chirp SonicLib to control ultrasonic sensors in an embedded C 
 * application.
 *
 * It configures connected CH101 or CH201 sensors, sets up a measurement 
 * timer, and triggers the sensors each time the timer expires.
 * On completion of each measurement, it reads out the sensor data and 
 * prints it over the console serial port.
 *
 * The settings used to configure the sensors are defined in
 * the app_config.h header file. 
 *
 ***********************************************************************/

/*
 Copyright ?2016-2020, Chirp Microsystems.  All rights reserved.

 Chirp Microsystems CONFIDENTIAL

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 You can contact the authors of this program by email at support@chirpmicro.com
 or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
*/


/* Includes */
#include <stdio.h>
#include "soniclib.h"			// Chirp SonicLib sensor API definitions
#include "chirp_board_config.h"	// required header with basic device counts etc.
#include "app_config.h"
#include "app_version.h"
#include "chirp_bsp.h"			// board support package function definitions
#include "chirp_smartsonic.h"

/* Bit flags used in main loop to check for completion of sensor I/O.  */
#define DATA_READY_FLAG		(1 << 0)
#define IQ_READY_FLAG		(1 << 1)

/* chirp_data_t - Structure to hold measurement data for one sensor
 *   This structure is used to hold the data from one measurement cycle from 
 *   a sensor.  The data values include the measured range, the ultrasonic 
 *   signal amplitude, the number of valid samples (I/Q data pairs) in the 
 *   measurement, and (optionally) the full amplitude data and/or raw I/Q data 
 *   from the measurement.
 *
 *  The format of this data structure is specific to this application, so 
 *  you may change it as desired.
 *
 *  A "chirp_data[]" array of these structures, one for each possible sensor, 
 *  is declared in the main.c file.  The sensor's device number is 
 *  used to index the array.
 */
typedef struct {
	uint32_t		range;						// from ch_get_range()
	uint16_t		amplitude;					// from ch_get_amplitude()
	uint16_t		num_samples;				// from ch_get_num_samples()
#ifdef READ_AMPLITUDE_DATA
	uint16_t		amp_data[DATA_MAX_NUM_SAMPLES];	
												// from ch_get_amplitude_data()
#endif
#ifdef READ_IQ_DATA
	ch_iq_sample_t	iq_data[DATA_MAX_NUM_SAMPLES];	
												// from ch_get_iq_data()
#endif
} chirp_data_t;


/* Array of structs to hold measurement data, one for each possible device */
chirp_data_t	chirp_data[CHIRP_MAX_NUM_SENSORS];		

/* Array of ch_dev_t device descriptors, one for each possible device */
ch_dev_t	chirp_devices[CHIRP_MAX_NUM_SENSORS];		

/* Configuration structure for group of sensors */
ch_group_t 	chirp_group;							

/* Detection level settings - for CH201 sensors only
 *   Each threshold entry includes the starting sample number & threshold level.
 */
ch_thresholds_t chirp_ch201_thresholds = {
	.threshold = {
		{0, 	5000},		/* threshold 0 */
		{26,	2000},		/* threshold 1 */
		{39,	800},		/* threshold 2 */
		{56,	400},		/* threshold 3 */
		{79,	250},		/* threshold 4 */
		{89,	175}		/* threshold 5 */
	}
};


/* Task flag word
 *   This variable contains the DATA_READY_FLAG and IQ_READY_FLAG bit flags 
 *   that are set in I/O processing routines.  The flags are checked in the 
 *   main() loop and, if set, will cause an appropriate handler function to 
 *   be called to process sensor data.  
 */
volatile uint32_t taskflags = 0;

/* Device tracking variables
 *   These are bit-field variables which contain a separate bit assigned to
 *   each (possible) sensor, indexed by the device number.  The active_devices
 *   variable contains the bit pattern describing which ports have active
 *   sensors connected.  The data_ready_devices variable is set bit-by-bit
 *   as sensors interrupt, indicating they have completed a measurement
 *   cycle.  The two variables are compared to determine when all active
 *   devices have interrupted.
 */
static uint32_t active_devices;
static uint32_t data_ready_devices;

/* Number of connected sensors */
static uint8_t	num_connected_sensors = 0;

/* Number of sensors that use h/w triggering to start measurement */
static uint8_t	num_triggered_devices = 0;

#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
/* Count of non-blocking I/Q reads queued */
static uint8_t	num_io_queued = 0;
#endif


/* Forward declarations */
static void    sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num);
static void    io_complete_callback(ch_group_t *grp_ptr);
static uint8_t display_config_info(ch_dev_t *dev_ptr);
static uint8_t handle_data_ready(ch_group_t *grp_ptr);

#ifdef READ_IQ_DATA
static uint8_t display_iq_data(ch_dev_t *dev_ptr);
#ifdef READ_IQ_NONBLOCKING
static uint8_t handle_iq_data_done(ch_group_t *grp_ptr);
#endif
#endif


/* main() - entry point and main loop
 *
 * This function contains the initialization sequence for the application
 * on the board, including system hardware initialization, sensor discovery
 * and configuration, callback routine registration, and timer setup.  After 
 * the initialization sequence completes, this routine enters an infinite 
 * loop that will run for the remainder of the application execution.
 */

int main(void) {
	ch_group_t	*grp_ptr = &chirp_group;
	uint8_t chirp_error = 0;
	uint8_t num_ports;
	uint8_t dev_num;

	/* Initialize board hardware functions 
	 *   This call to the board support package (BSP) performs all necessary 
	 *   hardware initialization for the application to run on this board.
	 *   This includes setting up memory regions, initializing clocks and 
	 *   peripherals (including I2C and serial port), and any processor-specific
	 *   startup sequences.
	 *
	 *   The chbsp_board_init() function also initializes fields within the 
	 *   sensor group descriptor, including number of supported sensors and 
	 *   the RTC clock calibration pulse length.
	 */
	chbsp_board_init(grp_ptr);

	printf("\n\nHello Chirp! - Chirp SonicLib Example Application\n");
	printf("    Compile time:  %s %s\n", __DATE__, __TIME__);
	printf("    Version: %u.%u.%u", APP_VERSION_MAJOR, APP_VERSION_MINOR,
										  APP_VERSION_REV);
	printf("    SonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR, 
										  SONICLIB_VER_MINOR, SONICLIB_VER_REV);
	printf("\n");


	/* Get the number of (possible) sensor devices on the board
	 *   Set by the BSP during chbsp_board_init() 
	 */
	num_ports = ch_get_num_ports(grp_ptr);


	/* Initialize sensor descriptors.
	 *   This loop initializes each (possible) sensor's ch_dev_t descriptor, 
	 *   although we don't yet know if a sensor is actually connected.
	 *
	 *   The call to ch_init() specifies the sensor descriptor, the sensor group
	 *   it will be added to, the device number within the group, and the sensor
	 *   firmware initialization routine that will be used.  (The sensor 
	 *   firmware selection effectively specifies whether it is a CH101 or 
	 *   CH201 sensor, as well as the exact feature set.)
	 */
	printf("Initializing sensor(s)... ");

	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		ch_dev_t *dev_ptr = &(chirp_devices[dev_num]);	// init struct in array

		/* Init device descriptor 
		 *   Note that this assumes all sensors will use the same sensor 
		 *   firmware.  The CHIRP_SENSOR_FW_INIT_FUNC symbol is defined in 
		 *   app_config.h and is used for all devices.
		 *
		 *   However, it is possible for different sensors to use different 
		 *   firmware images, by specifying different firmware init routines 
		 *   when ch_init() is called for each.
		 */
		chirp_error |= ch_init(dev_ptr, grp_ptr, dev_num, 
							   CHIRP_SENSOR_FW_INIT_FUNC);
	}

	/* Start all sensors.
	 *   The ch_group_start() function will search each port (that was 
	 *   initialized above) for a sensor. If it finds one, it programs it (with
	 *   the firmware specified above during ch_init()) and waits for it to 
	 *   perform a self-calibration step.  Then, once it has found all the 
	 *   sensors, ch_group_start() completes a timing reference calibration by 
	 *   applying a pulse of known length to the sensor's INT line.
	 */
	if (chirp_error == 0) {
		printf("starting group... ");
		chirp_error = ch_group_start(grp_ptr);
	}

	if (chirp_error == 0) {
		printf("OK\n");
	} else {
		printf("FAILED: %d\n", chirp_error);
	}
	printf("\n");

	/* Get and display the initialization results for each connected sensor.
	 *   This loop checks each device number in the sensor group to determine 
	 *   if a sensor is actually connected.  If so, it makes a series of 
	 *   function calls to get different operating values, including the 
	 *   operating frequency, clock calibration values, and firmware version.
 	 */
	printf("Sensor\tType \t   Freq\t\t RTC Cal \tFirmware\n");

	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			printf("%d\tCH%d\t %u Hz\t%u@%ums\t%s\n", dev_num,
									ch_get_part_number(dev_ptr),
									(unsigned int) ch_get_frequency(dev_ptr),
									ch_get_rtc_cal_result(dev_ptr),
									ch_get_rtc_cal_pulselength(dev_ptr),
									ch_get_fw_version_string(dev_ptr));
		}
	}
	printf("\n");

	/* Register callback function to be called when Chirp sensor interrupts */
	ch_io_int_callback_set(grp_ptr, sensor_int_callback);

	/* Register callback function called when non-blocking I/Q readout completes
 	 *   Note, this callback will only be used if READ_IQ_DATA_NONBLOCK is 
	 *   defined to enable non-blocking I/Q readout in this application.
	 */
	ch_io_complete_callback_set(grp_ptr, io_complete_callback);

	/* Configure each sensor with its operating parameters 
	 *   Initialize a ch_config_t structure with values defined in the
	 *   app_config.h header file, then write the configuration to the 
	 *   sensor using ch_set_config().
	 */
	printf ("Configuring sensor(s)...\n");
	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		ch_config_t dev_config;
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {

			/* Select sensor mode 
			 *   All connected sensors are placed in hardware triggered mode.
 	 		 *   The first connected (lowest numbered) sensor will transmit and 
			 *   receive, all others will only receive.
 	 		 */

			num_connected_sensors++;			// count one more connected
			active_devices |= (1 << dev_num);	// add to active device bit mask
			
			if (num_connected_sensors == 1) {	// if this is the first sensor
				dev_config.mode = CHIRP_FIRST_SENSOR_MODE;
			} else {									
				dev_config.mode = CHIRP_OTHER_SENSOR_MODE;
			}

			if (dev_config.mode != CH_MODE_FREERUN) {	// unless free-running
				num_triggered_devices++;				// will be triggered
			}

			/* Init config structure with values from app_config.h */
			dev_config.max_range       = CHIRP_SENSOR_MAX_RANGE_MM;
			dev_config.static_range    = CHIRP_SENSOR_STATIC_RANGE;

			/* If sensor will be free-running, set internal sample interval */
			if (dev_config.mode == CH_MODE_FREERUN) {
				dev_config.sample_interval = MEASUREMENT_INTERVAL_MS;
			} else {
				dev_config.sample_interval = 0;
			}

			/* Set detection thresholds (CH201 only) */
			if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER) {
				/* Set pointer to struct containing detection thresholds */
				dev_config.thresh_ptr = &chirp_ch201_thresholds;	
			} else {
				dev_config.thresh_ptr = 0;							
			}

			/* Apply sensor configuration */
			chirp_error = ch_set_config(dev_ptr, &dev_config);

			/* Enable sensor interrupt if using free-running mode 
			 *   Note that interrupt is automatically enabled if using 
			 *   triggered modes.
			 */
			if ((!chirp_error) && (dev_config.mode == CH_MODE_FREERUN)) {
				chbsp_set_io_dir_in(dev_ptr);
				chbsp_io_interrupt_enable(dev_ptr);
			}

			/* Read back and display config settings */
			if (!chirp_error) {
				display_config_info(dev_ptr);
			} else {
				printf("Device %d: Error during ch_set_config()\n", dev_num);
			}

			/* Turn on an LED to indicate device connected */
			if (!chirp_error) {
				chbsp_led_on(dev_num);
			}
		}
	}

	printf("\n");

	/* Enable receive sensor pre-triggering, if specified */
	ch_set_rx_pretrigger(grp_ptr, RX_PRETRIGGER_ENABLE);

	/* Initialize the periodic timer we'll use to trigger the measurements.
 	 *   This function initializes a timer that will interrupt every time it 
	 *   expires, after the specified measurement interval.  The function also 
	 *   registers a callback function that will be called from the timer 
	 *   handler when the interrupt occurs.  The callback function will be 
	 *   used to trigger a measurement cycle on the group of sensors.
	 */
	if (num_triggered_devices > 0) {
		printf("Initializing sample timer for %dms interval... ", 
				MEASUREMENT_INTERVAL_MS);

		chbsp_periodic_timer_init(MEASUREMENT_INTERVAL_MS, 
							      periodic_timer_callback);

		/* Enable interrupt and start timer to trigger sensor sampling */
		chbsp_periodic_timer_irq_enable();
		chbsp_periodic_timer_start();

		printf("OK\n");
	}

	printf("Starting measurements\n");

	/* Enter main loop 
	 *   This is an infinite loop that will run for the remainder of the system 
	 *   execution.  The processor is put in a low-power sleep mode between 
	 *   measurement cycles and is awakened by interrupt events.  
	 *
	 *   The interrupt may be the periodic timer (set up above), a data-ready 
	 *   interrupt from a sensor, or the completion of a non-blocking I/O 
	 *   operation.  This loop will check flags that are set during the 
	 *   callback functions for the data-ready interrupt and the non-blocking 
	 *   I/O complete.  Based on the flags that are set, this loop will call 
	 *   the appropriate routines to handle and/or display sensor data.
	 */
	while (1) {		/* LOOP FOREVER */
		/*
		 * Put processor in light sleep mode if there are no pending tasks, but 
		 * never turn off the main clock, so that interrupts can still wake 
		 * the processor.
		 */
		if (taskflags==0) {
			chbsp_proc_sleep();			// put processor in low-power sleep mode

			/* We only continue here after an interrupt wakes the processor */
		}

		/* Check for sensor data-ready interrupt(s) */
		if (taskflags & DATA_READY_FLAG) {

			/* All sensors have interrupted - handle sensor data */
			taskflags &= ~DATA_READY_FLAG;		// clear flag
			handle_data_ready(grp_ptr);			// read and display measurement
		}

#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
		/* Start any pending non-blocking I/Q reads */
		if (num_io_queued != 0) {
			ch_io_start_nb(grp_ptr);
			num_io_queued = 0;
		}

		/* Check for non-blocking I/Q read complete */
		if (taskflags & IQ_READY_FLAG) {

			/* All non-blocking I/Q readouts have completed */
			taskflags &= ~IQ_READY_FLAG;		// clear flag
			handle_iq_data_done(grp_ptr);		// display I/Q data
		}
#endif

	}	// end  while(1) main loop
}


/*
 * periodic_timer_callback() - periodic timer callback routine
 *
 * This function is called by the periodic timer interrupt when the timer 
 * expires.  Because the periodic timer is used to initiate a new measurement 
 * cycle on a group of sensors, this function calls ch_group_trigger() during 
 * each execution.
 *
 * This function is registered by the call to chbsp_periodic_timer_init() 
 * in main().
 */

void periodic_timer_callback(void) {

	if (num_triggered_devices > 0) {
		ch_group_trigger(&chirp_group);
	}
}


/*
 * sensor_int_callback() - sensor interrupt callback routine
 *
 * This function is called by the board support package's interrupt handler for 
 * the sensor's INT line every time that the sensor interrupts.  The device 
 * number parameter, dev_num, is used to identify the interrupting device
 * within the sensor group.  (Generally the device number is same as the port 
 * number used in the BSP to manage I/O pins, etc.)
 *
 * Each time this function is called, a bit is set in the data_ready_devices
 * variable to identify the interrupting device.  When all active sensors have
 * interrupted (found by comparing with the active_devices variable), the
 * DATA_READY_FLAG is set.  That flag will be detected in the main() loop.
 *
 * This callback function is registered by the call to ch_io_int_callback_set() 
 * in main().
 */
static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num) {
	ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

	data_ready_devices |= (1 << dev_num);		// add to data-ready bit mask

	if (data_ready_devices == active_devices) {
		/* All active sensors have interrupted after performing a measurement */
		data_ready_devices = 0;

		/* Set data-ready flag - it will be checked in main() loop */
		taskflags |= DATA_READY_FLAG;

		/* Disable interrupt unless in free-running mode
		 *   It will automatically be re-enabled by the next ch_group_trigger() 
		 */
		if (ch_get_mode(dev_ptr) == CH_MODE_FREERUN) {
			chbsp_set_io_dir_in(dev_ptr);				// set INT line as input
			chbsp_group_io_interrupt_enable(grp_ptr);
		} else {
			chbsp_group_io_interrupt_disable(grp_ptr);
		}
	}
}


/*
 * io_complete_callback() - non-blocking I/O complete callback routine
 *
 * This function is called by SonicLib's I2C DMA handling function when all 
 * outstanding non-blocking I/Q readouts have completed.  It simply sets a flag 
 * that will be detected and handled in the main() loop.
 *
 * This callback function is registered by the call to 
 * ch_io_complete_callback_set() in main().
 *
 *  Note: This callback is only used if READ_IQ_NONBLOCKING is defined to 
 *  select non-blocking I/Q readout in this application.
 */
static void io_complete_callback(ch_group_t __attribute__((unused)) *grp_ptr) {

	taskflags |= IQ_READY_FLAG;
}


/*
 * display_config_info() - display the configuration values for a sensor
 *
 * This function displays the current configuration settings for an individual 
 * sensor.  The operating mode, maximum range, and static target rejection 
 * range (if used) are displayed.
 *
 * For CH201 sensors only, the multiple detection threshold values are also 
 * displayed.
 */
static uint8_t display_config_info(ch_dev_t *dev_ptr) {
	ch_config_t 	read_config;
	uint8_t 		chirp_error;
	uint8_t 		dev_num = ch_get_dev_num(dev_ptr);

	/* Read configuration values for the device into ch_config_t structure */
	chirp_error = ch_get_config(dev_ptr, &read_config);

	if (!chirp_error) {
		const char *mode_string;

		switch (read_config.mode) {
			case CH_MODE_IDLE:
				mode_string = "IDLE";
				break;
			case CH_MODE_FREERUN:
				mode_string = "FREERUN";
				break;
			case CH_MODE_TRIGGERED_TX_RX:
				mode_string = "TRIGGERED_TX_RX";
				break;
			case CH_MODE_TRIGGERED_RX_ONLY:
				mode_string = "TRIGGERED_RX_ONLY";
				break;
			default:
				mode_string = "UNKNOWN";
		}

		/* Display sensor number, mode and max range */
		printf("Sensor %d:\tmax_range=%dmm \tmode=%s  ", dev_num,
				read_config.max_range, mode_string);

		/* Display static target rejection range, if used */
		if (read_config.static_range != 0) {
			printf("static_range=%d samples", read_config.static_range);
		}
	
		/* Display detection thresholds (only supported on CH201) */
		if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER) {
			ch_thresholds_t read_thresholds;

			/* Get threshold values in structure */
			chirp_error = ch_get_thresholds(dev_ptr, &read_thresholds);

			if (!chirp_error) {
				printf("\n  Detection thresholds:\n");
				for (int i = 0; i < CH_NUM_THRESHOLDS; i++) {
					printf("     %d\tstart: %2d\tlevel: %d\n", i, 
							read_thresholds.threshold[i].start_sample,
							read_thresholds.threshold[i].level);
				}
			} else {
				printf(" Device %d: Error during ch_get_thresholds()", dev_num);
			}
		}
		printf("\n");

	} else {
		printf(" Device %d: Error during ch_get_config()\n", dev_num);
	}

	return chirp_error;
}


/*
 * handle_data_ready() - get and display data from all sensors
 *
 * This routine is called from the main() loop after all sensors have 
 * interrupted. It shows how to read the sensor data once a measurement is 
 * complete.  This routine always reads out the range and amplitude, and 
 * optionally will read out the amplitude data or raw I/Q for all samples
 * in the measurement.
 *
 * See the comments in app_config.h for information about the amplitude data
 * and I/Q readout build options.
 *
 */
static uint8_t handle_data_ready(ch_group_t *grp_ptr) {
	uint8_t 	dev_num;
	int 		num_samples = 0;
	uint8_t 	ret_val = 0;

	/* Read and display data from each connected sensor 
	 *   This loop will write the sensor data to this application's "chirp_data"
	 *   array.  Each sensor has a separate chirp_data_t structure in that 
	 *   array, so the device number is used as an index.
	 */

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {

			/* Get measurement results from each connected sensor 
			 *   For sensor in transmit/receive mode, report one-way echo 
			 *   distance,  For sensor(s) in receive-only mode, report direct 
			 *   one-way distance from transmitting sensor 
			 */
			
			if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY) {
				chirp_data[dev_num].range = ch_get_range(dev_ptr, 
														CH_RANGE_DIRECT);
			} else {
				chirp_data[dev_num].range = ch_get_range(dev_ptr, 
														CH_RANGE_ECHO_ONE_WAY);
			}

			if (chirp_data[dev_num].range == CH_NO_TARGET) {
				/* No target object was detected - no range value */

				chirp_data[dev_num].amplitude = 0;  /* no updated amplitude */

				printf("Port %d:          no target found        ", dev_num);

			} else {
				/* Target object was successfully detected (range available) */

				 /* Get the new amplitude value - it's only updated if range 
				  * was successfully measured.  */
				chirp_data[dev_num].amplitude = ch_get_amplitude(dev_ptr);

				printf("Port %d:  Range: %0.1f mm  Amp: %u  ", dev_num, 
						(float) chirp_data[dev_num].range/32.0f,
					   	chirp_data[dev_num].amplitude);
			}

			/* Store number of active samples in this measurement */
			num_samples = ch_get_num_samples(dev_ptr);
			chirp_data[dev_num].num_samples = num_samples;

			/* Optionally read amplitude values for all samples */
#ifdef READ_AMPLITUDE_DATA
			uint16_t 	start_sample = 0;
			ch_get_amplitude_data(dev_ptr, chirp_data[dev_num].amp_data, 
								  start_sample, num_samples, CH_IO_MODE_BLOCK);

#ifdef OUTPUT_AMPLITUDE_DATA
			printf("\n");
			for (uint8_t count = 0; count < num_samples; count++) {

				printf("%d\n",  chirp_data[dev_num].amp_data[count]);
			}
#endif
#endif

			/* Optionally read raw I/Q values for all samples */
#ifdef READ_IQ_DATA
			display_iq_data(dev_ptr);
#endif
			/* If more than 2 sensors, put each on its own line */
			if (num_connected_sensors > 2) {
				printf("\n");
			}
		}
	}
	printf("\n");
	
	return ret_val;
}


#ifdef READ_IQ_DATA
/* display_iq_data() - Read full I/Q data from device into buffer or queue read
 * This function is used to obtain the full I/Q data from the sensor following a
 * measurement.  Depending on build options, it will either read the I/Q data
 * here, before returning, or it will queue a non-blocking read of the data.
 *
 * If READ_IQ_BLOCKING is defined, this function will read the data from 
 * the sensor into the application's "chirp_data" structure for this device 
 * before returning.  
 *
 * The I/Q data may be output in comma-separated-value (CSV) format, based on 
 * the OUTPUT_IQ_DATA_CSV build symbol.  If OUTPUT_IQ_DATA_CSV is defined, 
 * this function will output the full I/Q data as a series of comma-separated 
 * value pairs (Q, I), each on a separate line.  This may be a useful step 
 * toward making the data available in an external application for analysis 
 * (e.g. by copying the CSV values into a spreadsheet program).
 *
 * If READ_IQ_NONBLOCKING is defined, the I/Q read operation will be queued
 * and this routine will return immediately.  A callback routine will be called
 * when the read operation is complete.  The callback routine must have been 
 * registered using the ch_io_complete_callback_set function.
 */
uint8_t display_iq_data(ch_dev_t *dev_ptr) {
	uint16_t 	start_sample = 0;
	uint8_t 	error = 0;
	uint16_t 	num_samples = ch_get_num_samples(dev_ptr);
	uint8_t		dev_num = ch_get_dev_num(dev_ptr);

#if defined(READ_IQ_BLOCKING)
	/* Reading I/Q data in normal, blocking mode */

	error = ch_get_iq_data(dev_ptr, chirp_data[dev_num].iq_data, 
								start_sample, num_samples, CH_IO_MODE_BLOCK);

	if (!error) {
		printf("     %d I/Q samples copied", num_samples);

#ifdef OUTPUT_IQ_DATA_CSV
		/* Output IQ values in CSV format, one pair (sample) per line */
		ch_iq_sample_t *iq_ptr;

		iq_ptr = (ch_iq_sample_t *) &(chirp_data[dev_num].iq_data);

		for (int count = 0; count < num_samples; count++) {
			printf("\n%d,%d", iq_ptr->q, iq_ptr->i);	// output Q before I
			iq_ptr++;
		}
#endif
	} else {
		printf("     Error reading %d I/Q samples", num_samples);
	}

#elif defined(READ_IQ_NONBLOCKING)
	/* Reading I/Q data in non-blocking mode - queue a read operation */

	printf("     queuing %d I/Q samples... ", num_samples);

	error = ch_get_iq_data(dev_ptr, chirp_data[dev_num].iq_data, 
								start_sample, num_samples, CH_IO_MODE_NONBLOCK);

	if (!error) {
		num_io_queued++;		// record a pending non-blocking read
		printf("OK");
	} else {
		printf("**ERROR**");
	}
#endif  // defined(READ_IQ_BLOCKING)

	return error;
}
#endif  // READ_IQ_DATA


#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
/*
 * handle_iq_data_done() - handle raw I/Q data from a non-blocking read
 *
 * This function is called from the main() loop when a queued non-blocking 
 * readout of the raw I/Q data has completed for all sensors.  The data will 
 * have been placed in this application's "chirp_data" array, in the 
 * chirp_data_t structure for each sensor, indexed by the device number.  
 *
 * By default, this function takes no action on the I/Q data, except to display 
 * the number of samples that were read from the device.
 *
 * Optionally, if the OUTPUT_IQ_DATA_CSV build symbol is defined, this function 
 * will output the full I/Q data as a series of comma-separated value pairs 
 * (Q, I), each on a separate line.  This may be a useful step toward making 
 * the data available in an external application for analysis (e.g. by copying 
 * the CSV values into a spreadsheet program).
 */
static uint8_t handle_iq_data_done(ch_group_t *grp_ptr) {
	int				dev_num;
	uint16_t 		num_samples;
#ifdef OUTPUT_IQ_DATA_CSV
	ch_iq_sample_t 	*iq_ptr;			// pointer to an I/Q sample pair
#endif

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {

			num_samples = ch_get_num_samples(dev_ptr);

			printf ("Read %d samples from device %d:\n", num_samples, dev_num);

#ifdef OUTPUT_IQ_DATA_CSV
			iq_ptr = chirp_data[dev_num].iq_data;	// start of I/Q in data buf

			/* Output IQ values in CSV format, one pair per line */
			for (uint8_t count = 0; count < num_samples; count++) {

				printf("%d,%d\n", iq_ptr->q, iq_ptr->i);	// output Q before I
				iq_ptr++;									// next sample
			}
			printf("\n");
#endif
		}
	}

	return 0;
}

#endif  // (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))

