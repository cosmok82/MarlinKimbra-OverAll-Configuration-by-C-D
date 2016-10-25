/**
 * Configuration_Overall.h
 * Here you can define all your custom settings and they will overwrite configurations in the main configuration files.
 */

/******************************************
 * Author: Cosimo Orlando.				  *
 * Blog: http://creativityslashdesign.com *
 *										  *
 * Creative Commons License (NC)		  *
 ******************************************/

/***********************
 * Configuration_Basic *
 ***********************/
#define MECHANISM MECH_DELTA
 
/***********************
 * Configuration_Delta *
 ***********************/
#define KNOWN_MECH															// C/D edited
#define CUSTOM_MACHINE_NAME "C/D.com Kossel" 								// C/D edited
#define DELTA_SEGMENTS_PER_SECOND 160.0 									// C/D edited
#define DELTA_DIAGONAL_ROD 217.0 											// C/D edited
#define DELTA_SMOOTH_ROD_OFFSET 151.0 										// C/D edited
#define DELTA_EFFECTOR_OFFSET 22.5 											// C/D edited
#define DELTA_CARRIAGE_OFFSET 22.0 											// C/D edited
#define DELTA_PRINTABLE_RADIUS 75.0 										// C/D edited
#define TOWER_A_ENDSTOP_ADJ 0 												// C/D edited
#define TOWER_B_ENDSTOP_ADJ 0 												// C/D edited
#define TOWER_C_ENDSTOP_ADJ 0 												// C/D edited
#define TOWER_A_POSITION_ADJ 0 												// C/D edited
#define TOWER_B_POSITION_ADJ 0 												// C/D edited
#define TOWER_C_POSITION_ADJ 0 												// C/D edited
#define TOWER_A_RADIUS_ADJ 0 												// C/D edited
#define TOWER_B_RADIUS_ADJ 0 												// C/D edited
#define TOWER_C_RADIUS_ADJ 0 												// C/D edited
#define TOWER_A_DIAGROD_ADJ 0 												// C/D edited
#define TOWER_B_DIAGROD_ADJ 0 												// C/D edited
#define TOWER_C_DIAGROD_ADJ 0 												// C/D edited
#undef ENDSTOPPULLUPS 														// C/D edited
#define ENDSTOPPULLUP_XMIN 													// C/D edited
#define ENDSTOPPULLUP_YMIN 													// C/D edited
#define ENDSTOPPULLUP_ZMIN 													// C/D edited
#define ENDSTOPPULLUP_Z2MIN 												// C/D edited
#define ENDSTOPPULLUP_XMAX 													// C/D edited
#define ENDSTOPPULLUP_YMAX 													// C/D edited
#define ENDSTOPPULLUP_ZMAX 													// C/D edited
#define ENDSTOPPULLUP_Z2MAX 												// C/D edited
#define ENDSTOPPULLUP_ZPROBE 												// C/D edited
#define ENDSTOPPULLUP_EMIN 													// C/D edited
#define X_MIN_ENDSTOP_LOGIC false 											// C/D edited
#define Y_MIN_ENDSTOP_LOGIC false 											// C/D edited
#define Z_MIN_ENDSTOP_LOGIC false 											// C/D edited
#define Z2_MIN_ENDSTOP_LOGIC false 											// C/D edited
#define X_MAX_ENDSTOP_LOGIC false 											// C/D edited
#define Y_MAX_ENDSTOP_LOGIC false 											// C/D edited
#define Z_MAX_ENDSTOP_LOGIC false 											// C/D edited
#define Z2_MAX_ENDSTOP_LOGIC false 											// C/D edited
#define Z_PROBE_ENDSTOP_LOGIC false 										// C/D edited
#define E_MIN_ENDSTOP_LOGIC false 											// C/D edited
#define Z_ENDSTOP_SERVO_NR -1 												// C/D edited
#define Z_ENDSTOP_SERVO_ANGLES {90,0} 										// C/D edited // Z Servo Deploy and Stow angles
//#define Z_PROBE_FIX_MOUNTED                   							// C/D edited
//#define BLTOUCH                               							// C/D edited
//#define Z_PROBE_ALLEN_KEY                     							// C/D edited
#define Z_PROBE_DEPLOY_START_LOCATION {0,0,30,0}							// C/D edited
#define Z_PROBE_DEPLOY_END_LOCATION {0,0,30,0} 								// C/D edited
#define Z_PROBE_RETRACT_START_LOCATION {0,0,30,0}							// C/D edited
#define Z_PROBE_RETRACT_END_LOCATION {0,0,30,0} 							// C/D edited
#define X_PROBE_OFFSET_FROM_NOZZLE 0            							// C/D edited
#define Y_PROBE_OFFSET_FROM_NOZZLE 0            							// C/D edited
#define Z_PROBE_OFFSET_FROM_NOZZLE -1           							// C/D edited
#define XY_PROBE_SPEED 10000 												// C/D edited
#define Z_PROBE_SPEED 3000 													// C/D edited
#define Z_RAISE_PROBE_DEPLOY_STOW 30 										// C/D edited
#define Z_RAISE_BETWEEN_PROBINGS 10 										// C/D edited
#define Z_PROBE_OFFSET_RANGE_MIN -50 										// C/D edited
#define Z_PROBE_OFFSET_RANGE_MAX  50 										// C/D edited
#define X_HOME_DIR 1														// C/D edited
#define Y_HOME_DIR 1                            							// C/D edited
#define Z_HOME_DIR 1                            							// C/D edited
#define E_HOME_DIR -1                           							// C/D edited
#define X_ENABLE_ON 0                           							// C/D edited
#define Y_ENABLE_ON 0                           							// C/D edited
#define Z_ENABLE_ON 0                           							// C/D edited
#define E_ENABLE_ON 0                           							// C/D edited
#define INVERT_X_STEP_PIN false                 							// C/D edited
#define INVERT_Y_STEP_PIN false                 							// C/D edited
#define INVERT_Z_STEP_PIN false                 							// C/D edited
#define INVERT_E_STEP_PIN false                 							// C/D edited
#define INVERT_X_DIR false                      							// C/D edited
#define INVERT_Y_DIR false                      							// C/D edited
#define INVERT_Z_DIR false                      							// C/D edited
#define INVERT_E0_DIR false                     							// C/D edited
#define INVERT_E1_DIR false                     							// C/D edited
#define INVERT_E2_DIR false                     							// C/D edited
#define INVERT_E3_DIR false                     							// C/D edited
#define INVERT_E4_DIR false                     							// C/D edited
#define INVERT_E5_DIR false                     							// C/D edited
#define DISABLE_X false                         							// C/D edited
#define DISABLE_Y false                         							// C/D edited
#define DISABLE_Z false                         							// C/D edited
#define DISABLE_E false                         							// C/D edited
#define DISABLE_INACTIVE_EXTRUDER false										// C/D edited
#define MANUAL_HOME_POSITIONS   											// C/D edited // If defined, MANUAL_*_HOME_POS below will be used
#define BED_CENTER_AT_0_0       											// C/D edited // If defined, the center of the bed is at (X=0, Y=0)
#define MANUAL_X_HOME_POS 0                     							// C/D edited
#define MANUAL_Y_HOME_POS 0                     							// C/D edited
#define MANUAL_Z_HOME_POS 293.30                							// C/D edited
#define X_MAX_POS DELTA_PRINTABLE_RADIUS        							// C/D edited
#define X_MIN_POS -DELTA_PRINTABLE_RADIUS       							// C/D edited
#define Y_MAX_POS DELTA_PRINTABLE_RADIUS        							// C/D edited
#define Y_MIN_POS -DELTA_PRINTABLE_RADIUS       							// C/D edited
#define Z_MAX_POS MANUAL_Z_HOME_POS             							// C/D edited
#define Z_MIN_POS 0															// C/D edited
#define E_MIN_POS 0															// C/D edited
#define AXIS_RELATIVE_MODES {false, false, false, false} 					// C/D edited
//#define AUTO_BED_LEVELING_FEATURE               							// C/D edited
//#define Z_PROBE_REPEATABILITY_TEST              							// C/D edited
#define AUTOCALIBRATION_PRECISION 0.1             							// C/D edited
#define AUTO_BED_LEVELING_GRID_POINTS 6           							// C/D edited
#define DEFAULT_AXIS_STEPS_PER_UNIT {80, 80, 80, 88.24, 150, 150, 150}  	// C/D edited // X,  Y,  Z,  E0...(per extruder)
#define DEFAULT_MAX_FEEDRATE {200, 200, 200, 200, 200, 200, 200} 			// C/D edited
#define MANUAL_FEEDRATE {50*60, 50*60, 50*60, 60} 							// C/D edited
#define DEFAULT_MINIMUMFEEDRATE 0.0           								// C/D edited // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE 0.0										// C/D edited
#define MINIMUM_PLANNER_SPEED 0.05          								// C/D edited // (mm/sec)
#define DEFAULT_MAX_ACCELERATION {3000, 3000, 3000, 3000, 3000, 3000, 3000} // C/D edited
#define DEFAULT_RETRACT_ACCELERATION {3000, 3000, 3000, 3000} 				// C/D edited
#define DEFAULT_ACCELERATION 3000 											// C/D edited
#define DEFAULT_TRAVEL_ACCELERATION 3000 									// C/D edited
#define DEFAULT_XYJERK 20.0 												// C/D edited
#define DEFAULT_ZJERK 20.0 													// C/D edited
#define DEFAULT_EJERK {20.0, 20.0, 20.0, 20.0} 								// C/D edited
#define HOMING_FEEDRATE_XYZ (50*60) 										// C/D edited
#define HOMING_FEEDRATE_E 0													// C/D edited
#define HOMING_FEEDRATE { HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_E } // C/D edited
/*  #define HOMING_FEEDRATE_X (50*60)                                       // C/D edited
	#define HOMING_FEEDRATE_Y (50*60)                                       // C/D edited
	#define HOMING_FEEDRATE_Z (2*60)                                        // C/D edited */
#define XYZ_HOME_BUMP_MM 5 													// C/D edited
#define XYZ_BUMP_DIVISOR 5 													// C/D edited
#define HOTEND_OFFSET_X {0.0, 0.0, 0.0, 0.0} 								// C/D edited
#define HOTEND_OFFSET_Y {0.0, 0.0, 0.0, 0.0} 								// C/D edited
#define HOTEND_OFFSET_Z {0.0, 0.0, 0.0, 0.0} 								// C/D edited

/*************************
 * Configuration_Feature *
 *************************/                                                 // C/D edited
//#define FAST_PWM_FAN                                                      // C/D edited
//#define FAN_SOFT_PWM                                                      // C/D edited
#define SOFT_PWM_SCALE 0 													// C/D edited
//#define FAN_KICKSTART_TIME 100                                            // C/D edited
//#define FAN_MIN_PWM 50                                                    // C/D edited
//#define CONTROLLERFAN                                                     // C/D edited
#define CONTROLLERFAN_SECS 60 												// C/D edited // How many seconds, after all motors were disabled, the fan should run
#define CONTROLLERFAN_SPEED 255	 											// C/D edited // 255 = full speed
#define CONTROLLERFAN_MIN_SPEED 0 											// C/D edited
//#define EXTRUDER_AUTO_FAN													// C/D edited
#define EXTRUDER_AUTO_FAN_TEMPERATURE 50 									// C/D edited
#define EXTRUDER_AUTO_FAN_SPEED 255 										// C/D edited
#define EXTRUDER_AUTO_FAN_MIN_SPEED 0 										// C/D edited
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75 									// C/D edited
#define PREVENT_DANGEROUS_EXTRUDE 											// C/D edited
#define EXTRUDE_MINTEMP 170  												// C/D edited // degC
//#define SINGLENOZZLE                                                      // C/D edited
//#define BARICUDA                                                          // C/D edited
//#define COLOR_MIXING_EXTRUDER                                             // C/D edited
#undef MIXING_STEPPERS                                                 		// C/D edited
#define MIXING_VIRTUAL_TOOLS 16 											// C/D edited
//#define MKR4                                                              // C/D edited
//#define INVERTED_RELE_PINS                                                // C/D edited
//#define MKR6                                                              // C/D edited
//#define INVERTED_RELE_PINS                                                // C/D edited
//#define NPR2																// C/D edited
#define COLOR_STEP {0,10,20,30} 											// C/D edited
#define COLOR_SLOWRATE 170            										// C/D edited // MICROSECOND delay for carter motor routine (Carter Motor Feedrate: upper value-slow feedrate)  
#define COLOR_HOMERATE 4              										// C/D edited // FEEDRATE for carter home
#define MOTOR_ANGLE 1.8               										// C/D edited // Nema angle for single step 
#define DRIVER_MICROSTEP 4            										// C/D edited // Microstep moltiplicator driver (set jumper MS1-2-3) off-on-off 1/4 microstepping.
#define CARTER_MOLTIPLICATOR 14.22    										// C/D edited // CARTER MOLTIPLICATOR (gear ratio 13/31-10/31)
//#define DONDOLO_SINGLE_MOTOR												// C/D edited
//#define DONDOLO_DUAL_MOTOR												// C/D edited
#define DONDOLO_SERVO_INDEX 0                                               // C/D edited
#define DONDOLO_SERVOPOS_E0 120                                             // C/D edited
#define DONDOLO_SERVOPOS_E1 10                                              // C/D edited
#define DONDOLO_SERVO_DELAY 1000                                            // C/D edited
//#define IDLE_OOZING_PREVENT                                               // C/D edited
#define IDLE_OOZING_MINTEMP 190												// C/D edited
#define IDLE_OOZING_FEEDRATE 50    											// C/D edited // default feedrate for retracting (mm/s)
#define IDLE_OOZING_SECONDS 5												// C/D edited
#define IDLE_OOZING_LENGTH 15    											// C/D edited // default retract length (positive mm)
#define IDLE_OOZING_RECOVER_LENGTH 0     									// C/D edited // default additional recover length (mm, added to retract length when recovering)
#define IDLE_OOZING_RECOVER_FEEDRATE 50    									// C/D edited // default feedrate for recovering from retraction (mm/s)
//#define EXTRUDER_RUNOUT_PREVENT                                           // C/D edited
#define EXTRUDER_RUNOUT_MINTEMP 190                                         // C/D edited
#define EXTRUDER_RUNOUT_SECONDS 30                                          // C/D edited
#define EXTRUDER_RUNOUT_ESTEPS 14  											// C/D edited // mm filament
#define EXTRUDER_RUNOUT_SPEED 1500  										// C/D edited // extrusion speed
#define EXTRUDER_RUNOUT_EXTRUDE 100                                         // C/D edited
#define EASY_LOAD 															// C/D edited // Easy filament loading feature
#define BOWDEN_LENGTH 650                                                   // C/D edited
#define LCD_PURGE_LENGTH 10                                                 // C/D edited
#define LCD_RETRACT_LENGTH 5                                                // C/D edited
#define LCD_PURGE_FEEDRATE 3                                                // C/D edited
#define LCD_RETRACT_FEEDRATE 5                                              // C/D edited
#define LCD_LOAD_FEEDRATE 20												// C/D edited
#define LCD_UNLOAD_FEEDRATE 20												// C/D edited
//#define ADVANCE                                                           // C/D edited
#define EXTRUDER_ADVANCE_K 0.0												// C/D edited
#define D_FILAMENT 1.75														// C/D edited
//#define LIN_ADVANCE                                                       // C/D edited
#define LIN_ADVANCE_K 75													// C/D edited
//#define ADVANCE_LPC														// C/D edited // other definition
#define ADVANCE_LPC_K 75													// C/D edited // other definition
#define FILAMENT_CHANGE_FEATURE  											// C/D edited // Easy filament changing feature
#define FILAMENT_CHANGE_X_POS 3            									// C/D edited // X position of hotend
#define FILAMENT_CHANGE_Y_POS 3            									// C/D edited // Y position of hotend
#define FILAMENT_CHANGE_Z_ADD 10           									// C/D edited // Z addition of hotend (lift)
#define FILAMENT_CHANGE_XY_FEEDRATE 100                                     // C/D edited
#define FILAMENT_CHANGE_Z_FEEDRATE 5                                        // C/D edited
#define FILAMENT_CHANGE_RETRACT_LENGTH 2                                    // C/D edited
#define FILAMENT_CHANGE_RETRACT_FEEDRATE 50                                 // C/D edited
#define FILAMENT_CHANGE_UNLOAD_LENGTH 100                                   // C/D edited
#define FILAMENT_CHANGE_UNLOAD_FEEDRATE 100                                 // C/D edited
#define FILAMENT_CHANGE_LOAD_LENGTH 100                                     // C/D edited
#define FILAMENT_CHANGE_LOAD_FEEDRATE 100                                   // C/D edited
#define FILAMENT_CHANGE_EXTRUDE_LENGTH 50                                   // C/D edited
#define FILAMENT_CHANGE_EXTRUDE_FEEDRATE 5                                  // C/D edited
#define FILAMENT_CHANGE_PRINTER_OFF 5                                       // C/D edited
#define SOFTWARE_MIN_ENDSTOPS true  										// C/D edited // If true, axis won't move to coordinates less than HOME_POS.
#define SOFTWARE_MAX_ENDSTOPS true  										// C/D edited // If true, axis won't move to coordinates greater than the defined lengths below.
#define ENDSTOPS_ONLY_FOR_HOMING											// C/D edited
//#define ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED								// C/D edited
#define ABORT_ON_ENDSTOP_HIT_INIT true										// C/D edited
#define MESH_MIN_X (X_MIN_POS + MESH_INSET)                                 // C/D edited
#define MESH_MAX_X (X_MAX_POS - (MESH_INSET))                               // C/D edited
#define MESH_MIN_Y (Y_MIN_POS + MESH_INSET)                                 // C/D edited
#define MESH_MAX_Y (Y_MAX_POS - (MESH_INSET))                               // C/D edited
//#define ENABLE_SERVOS                                                     // C/D edited
#define NUM_SERVOS 0														// C/D edited
//#define DEACTIVATE_SERVOS_AFTER_MOVE                                      // C/D edited
#define SERVO_DEACTIVATION_DELAY 300                                        // C/D edited
//#define Z_LATE_ENABLE                                                     // C/D edited
#define SLOWDOWN															// C/D edited
//#define QUICK_HOME                                                        // C/D edited
//#define HOME_Y_BEFORE_X                                                   // C/D edited
//#define BABYSTEPPING                                                      // C/D edited
#define BABYSTEP_XY  														// C/D edited // not only z, but also XY in the menu. more clutter, more functions
#define BABYSTEP_INVERT_Z false  											// C/D edited // true for inverse movements in Z
#define BABYSTEP_MULTIPLICATOR 2 											// C/D edited // faster z movements
//#define FWRETRACT //ONLY PARTIALLY TESTED									// C/D edited
#define MIN_RETRACT 0.1 													// C/D edited // minimum extruded mm to accept a automatic gcode retraction attempt
#define RETRACT_LENGTH 3 												  	// C/D edited // default retract length (positive mm)
#define RETRACT_LENGTH_SWAP 13  											// C/D edited // default swap retract length (positive mm), for extruder change
#define RETRACT_FEEDRATE 45 												// C/D edited // default feedrate for retracting (mm/s)
#define RETRACT_ZLIFT 0 													// C/D edited // default retract Z-lift
#define RETRACT_RECOVER_LENGTH 0   											// C/D edited // default additional recover length (mm, added to retract length when recovering)
#define RETRACT_RECOVER_LENGTH_SWAP 0   									// C/D edited // default additional swap recover length (mm, added to retract length when recovering from extruder change)
#define RETRACT_RECOVER_FEEDRATE 8   										// C/D edited // default feedrate for recovering from retraction (mm/s)
//#define DUAL_X_CARRIAGE													// C/D edited
#define X2_MIN_POS 80 														// C/D edited // set minimum to ensure second x-carriage doesn't hit the parked first X-carriage
#define X2_MAX_POS 353														// C/D edited // set maximum to the distance between toolheads when both heads are homed
#define X2_HOME_DIR 1 														// C/D edited // the second X-carriage always homes to the maximum endstop position
#define X2_HOME_POS X2_MAX_POS 												// C/D edited // default home position is the maximum carriage position
#define DEFAULT_DUAL_X_CARRIAGE_MODE 0                                      // C/D edited
#define TOOLCHANGE_PARK_ZLIFT 0.2      										// C/D edited // the distance to raise Z axis when parking an extruder
#define TOOLCHANGE_UNPARK_ZLIFT 1      										// C/D edited // the distance to raise Z axis when unparking an extruder
#define DEFAULT_DUPLICATION_X_OFFSET 100                                    // C/D edited
//#define X_DUAL_STEPPER_DRIVERS                                            // C/D edited
#undef INVERT_X2_VS_X_DIR                                          			// C/D edited
//#define Y_DUAL_STEPPER_DRIVERS											// C/D edited
#define INVERT_Y2_VS_Y_DIR false                                            // C/D edited
//#define Z_DUAL_STEPPER_DRIVERS                                            // C/D edited
//#define Z_DUAL_ENDSTOPS                                                   // C/D edited
//#define XY_FREQUENCY_LIMIT  15                                            // C/D edited
//#define SF_ARC_FIX														// C/D edited
//#define FILAMENT_SENSOR                                                   // C/D edited
#define FILAMENT_SENSOR_EXTRUDER_NUM 0                                      // C/D edited
#define MEASUREMENT_DELAY_CM         14     								// C/D edited // measurement delay in cm.  This is the distance from filament sensor to middle of barrel
#define MEASURED_UPPER_LIMIT 2.00											// C/D edited		
#define MEASURED_LOWER_LIMIT 1.35											// C/D edited 
#define MAX_MEASUREMENT_DELAY        20     								// C/D edited // delay buffer size in bytes (1 byte = 1cm)- limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)
#define DEFAULT_MEASURED_FILAMENT_DIA  DEFAULT_NOMINAL_FILAMENT_DIA  		// C/D edited // set measured to nominal initially
//#define FILAMENT_LCD_DISPLAY                                              // C/D edited
//#define FILAMENT_RUNOUT_SENSOR                                            // C/D edited
#define FILRUNOUT_PIN_INVERTING true										// C/D edited
//#undef ENDSTOPPULLUP_FIL_RUNOUT                                           // C/D edited
#define ENDSTOPPULLUP_FIL_RUNOUT                                            // C/D edited
#define FILAMENT_RUNOUT_SCRIPT "M600"                                       // C/D edited
//#define POWER_CONSUMPTION                                                 // C/D edited
#define POWER_VOLTAGE 12.00    												// C/D edited // (V) The power supply OUT voltage
#define POWER_SENSITIVITY 0.066 											// C/D edited // (V/A) How much increase V for 1A of increase
#define POWER_OFFSET 0.005   												// C/D edited // (A) Help to get 0A when no load is connected.
#define POWER_ZERO 2.500   													// C/D edited // (V) The /\V coming out from the sensor when no current flow.
#define POWER_ERROR 0.0     												// C/D edited // (%) Ammortize measure error.
#define POWER_EFFICIENCY 100.0     											// C/D edited // (%) The power efficency of the power supply
//#define POWER_CONSUMPTION_LCD_DISPLAY                                     // C/D edited
//#define FLOWMETER_SENSOR                                                  // C/D edited
#define FLOWMETER_MAXFLOW 6.0												// C/D edited // Liters per minute max
#define FLOWMETER_MAXFREQ 55   												// C/D edited // frequency of pulses at max flow
//#define MINFLOW_PROTECTION 4                                              // C/D edited
/**************************************************** EEPROM ************************************************************
 * The microcontroller can store settings in the EEPROM, e.g. max velocity...                                           *
 * M500 - stores parameters in EEPROM                                                                                   *
 * M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).                     *
 * M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to. *
 *                                                                                                                      *
 * Uncomment EEPROM SETTINGS to enable this feature.                                                                    *
 * Uncomment EEPROM CHITCHAT to enable EEPROM Serial responses.                                                         */
 /************************************************************************************************************************/
//#define EEPROM_SETTINGS                                                      // C/D edited
//#define EEPROM_CHITCHAT // Uncomment this to enable EEPROM Serial responses. // C/D edited
//#define DISABLE_M503                                                         // C/D edited
/************************************************************************************************************************/
//#define SDSUPPORT           												// C/D edited // Enable SD Card Support in Hardware Console
//#define SDSLOW              												// C/D edited // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SDEXTRASLOW         												// C/D edited // Use even slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SD_CHECK_AND_RETRY  												// C/D edited // Use CRC checks and retries on the SD communication
//#define SD_EXTENDED_DIR     												// C/D edited // Show extended directory including file length. Don't use this with Pronterface
//#define SD_DISABLED_DETECT                                                // C/D edited
//#define SD_DETECT_INVERTED                                                // C/D edited
#define SD_FINISHED_STEPPERRELEASE true  									// C/D edited // if sd support and the file is finished: disable steppers?
#define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E" 							// C/D edited // You might want to keep the z enabled so your bed stays in place.
#define SDCARD_RATHERRECENTFIRST  											// C/D edited // reverse file order of sd card menu display. Its sorted practically after the file system block order.
//#define MENU_ADDAUTOSTART                                                 // C/D edited
//#define SD_SETTINGS                 										// C/D edited // Uncomment to enable
#define SD_CFG_SECONDS 300         											// C/D edited // seconds between update
#define CFG_SD_FILE "INFO.CFG"  											// C/D edited // name of the configuration file
#define LCD_LANGUAGE en                                                     // C/D edited
#define DISPLAY_CHARSET_HD44780_JAPAN        								// C/D edited // this is the most common hardware
//#define DISPLAY_CHARSET_HD44780_WESTERN                                   // C/D edited
//#define DISPLAY_CHARSET_HD44780_CYRILLIC                                  // C/D edited
#define SHOW_BOOTSCREEN                                                     // C/D edited
//#define SHOW_CUSTOM_BOOTSCREEN                                            // C/D edited
//#define STRING_SPLASH_LINE1 "v" SHORT_BUILD_VERSION       				// C/D edited // will be shown during bootup in line 1
//#define STRING_SPLASH_LINE2 STRING_DISTRIBUTION_DATE      				// C/D edited // will be shown during bootup in line 2
#define SPLASH_SCREEN_DURATION 5000                       					// C/D edited // SPLASH SCREEN duration in millisecond
//#define LCD_SCREEN_ROT_90    												// C/D edited // Rotate screen orientation for graphics display by 90 degree clockwise
//#define LCD_SCREEN_ROT_180   												// C/D edited // Rotate screen orientation for graphics display by 180 degree clockwise
//#define LCD_SCREEN_ROT_270   												// C/D edited // Rotate screen orientation for graphics display by 270 degree clockwise
//#define INVERT_CLICK_BUTTON           									// C/D edited // Option for invert encoder button logic
//#define INVERT_BACK_BUTTON            									// C/D edited // Option for invert back button logic if avaible
//#define INVERT_ROTARY_SWITCH          									// C/D edited // Option for reverses the encoder direction for navigating LCD menus.
#define ENCODER_RATE_MULTIPLIER         									// C/D edited // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
#define ENCODER_10X_STEPS_PER_SEC   75  									// C/D edited // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
#define ENCODER_100X_STEPS_PER_SEC 160  									// C/D edited // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value
#define ULTIPANEL_FEEDMULTIPLY          									// C/D edited // Comment to disable setting feedrate multiplier via encoder
//#define ULTRA_LCD                              							// C/D edited // general LCD support, also 16x2
//#define DOGLCD                                 							// C/D edited // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)
//#define ENCODER_PULSES_PER_STEP 1              							// C/D edited // Increase if you have a high resolution encoder
//#define ENCODER_STEPS_PER_MENU_ITEM 5          							// C/D edited // Set according to ENCODER_PULSES_PER_STEP or your liking
//#define ULTIMAKERCONTROLLER                    							// C/D edited // As available from the Ultimaker online store.
//#define ULTIPANEL                              							// C/D edited // The UltiPanel as on Thingiverse
//#define SPEAKER                                							// C/D edited // The sound device is a speaker - not a buzzer. A buzzer resonates with his own frequency.
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100 							// C/D edited // the duration the buzzer plays the UI feedback sound. ie Screen Click
//#define LCD_FEEDBACK_FREQUENCY_HZ 1000         							// C/D edited // this is the tone frequency the buzzer plays when on UI feedback. ie Screen Click
//#define UI_VOLTAGE_LEVEL 0 												// C/D edited // 3.3 V
#define UI_VOLTAGE_LEVEL 1   												// C/D edited // 5 V
#define LCD_INFO_MENU                                                       // C/D edited
//#define RADDS_DISPLAY                                                     // C/D edited
//#define PANEL_ONE                                                         // C/D edited
//#define MAKRPANEL                                                         // C/D edited
//#define VIKI2                                                             // C/D edited
//#define miniVIKI                                                          // C/D edited
//#define ELB_FULL_GRAPHIC_CONTROLLER                                       // C/D edited
//#define SD_DETECT_INVERTED                                                // C/D edited
//#define REPRAPWORLD_GRAPHICAL_LCD                                         // C/D edited
#define REPRAP_DISCOUNT_SMART_CONTROLLER                                    // C/D edited
//#define G3D_PANEL                                                         // C/D edited
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER                     // C/D edited
//#define REPRAPWORLD_KEYPAD                                                // C/D edited
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0 								// C/D edited // how much should be moved when a key is pressed, eg 10.0 means 10mm per click
//#define RA_CONTROL_PANEL                                                  // C/D edited
//#define MINIPANEL                                                         // C/D edited
//#define NEXTION                                                           // C/D edited
#define NEXTION_SERIAL 1                                                    // C/D edited
#define NEXTION_PORT 1                                                      // C/D edited
//#define NEXTION_GFX                                                       // C/D edited
//#define LCD_I2C_SAINSMART_YWROBOT                                         // C/D edited
//#define LCD_I2C_PANELOLU2                                                 // C/D edited
//#define LCD_I2C_VIKI                                                      // C/D edited
//#define U8GLIB_SSD1306                                                    // C/D edited
//#define SAV_3DLCD                                                         // C/D edited
//#define USE_BIG_EDIT_FONT 												// C/D edited // We don't have a big font for Cyrillic, Kana (Needs 3120 bytes of PROGMEM)
//#define USE_SMALL_INFOFONT 												// C/D edited // Smaller font on the Info-screen (Needs 2300 bytes of PROGMEM)
#define LCD_PROGRESS_BAR                                                    // C/D edited
#define PROGRESS_BAR_BAR_TIME 5000                                          // C/D edited
#define PROGRESS_BAR_MSG_TIME 1500                                          // C/D edited
#define PROGRESS_MSG_EXPIRE 0                                               // C/D edited
//#define PROGRESS_MSG_ONCE        											// C/D edited // Uncomment this to show messages for MSG_TIME then hide them
//#define PHOTOGRAPH                                                        // C/D edited
//#define CHDK                                                              // C/D edited
#define CHDK_DELAY 50   													// C/D edited // How long in ms the pin should stay HIGH before going LOW again
//#define RFID_MODULE                                                       // C/D edited
#define RFID_SERIAL 1                                                       // C/D edited
//#define BLINKM                                                            // C/D edited
//#define LASERBEAM                                                         // C/D edited
#define DEFAULT_STEPPER_DEACTIVE_TIME 60                                    // C/D edited
//#define STEPPER_HIGH_LOW                                                  // C/D edited
#define STEPPER_HIGH_LOW_DELAY 1u  											// C/D edited // Delay in microseconds
//#define ENABLE_HIGH_SPEED_STEPPING                                        // C/D edited
//#define USE_MICROSTEPS                                                    // C/D edited
#define MICROSTEP_MODES {16,16,16,16}                                       // C/D edited
#define MOTOR_CURRENT {1.0,1.0,1.0,1.0,1.0,1.0,1.0}                         // C/D edited
#define DIGIPOT_MOTOR_CURRENT {135, 135, 135, 135, 135} 					// C/D edited // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)
//#define DIGIPOT_I2C                                                       // C/D edited
#define DIGIPOT_I2C_NUM_CHANNELS 8                                          // C/D edited
#define DIGIPOT_I2C_MOTOR_CURRENTS {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0} // C/D edited
//#define CONFIG_STEPPERS_TOSHIBA                                           // C/D edited
//#define HAVE_TMCDRIVER                                                    // C/D edited
//#define X_IS_TMC                                                          // C/D edited
#define X_MAX_CURRENT 1000  												// C/D edited // in mA
#define X_SENSE_RESISTOR 91 												// C/D edited // in mOhms
#define X_MICROSTEPS 16     												// C/D edited // number of microsteps
//#define X2_IS_TMC															// C/D edited
#define X2_MAX_CURRENT 1000 												// C/D edited // in mA
#define X2_SENSE_RESISTOR 91												// C/D edited // in mOhms
#define X2_MICROSTEPS 16    												// C/D edited // number of microsteps
//#define Y_IS_TMC															// C/D edited
#define Y_MAX_CURRENT 1000  												// C/D edited // in mA
#define Y_SENSE_RESISTOR 91 												// C/D edited // in mOhms
#define Y_MICROSTEPS 16     												// C/D edited // number of microsteps
//#define Y2_IS_TMC															// C/D edited
#define Y2_MAX_CURRENT 1000 												// C/D edited // in mA
#define Y2_SENSE_RESISTOR 91												// C/D edited // in mOhms
#define Y2_MICROSTEPS 16    												// C/D edited // number of microsteps 
//#define Z_IS_TMC															// C/D edited
#define Z_MAX_CURRENT 1000  												// C/D edited // in mA
#define Z_SENSE_RESISTOR 91 												// C/D edited // in mOhms
#define Z_MICROSTEPS 16     												// C/D edited // number of microsteps
//#define Z2_IS_TMC															// C/D edited
#define Z2_MAX_CURRENT 1000 												// C/D edited // in mA
#define Z2_SENSE_RESISTOR 91												// C/D edited // in mOhms
#define Z2_MICROSTEPS 16    												// C/D edited // number of microsteps
//#define E0_IS_TMC															// C/D edited
#define E0_MAX_CURRENT 1000 												// C/D edited // in mA
#define E0_SENSE_RESISTOR 91												// C/D edited // in mOhms
#define E0_MICROSTEPS 16    												// C/D edited // number of microsteps
//#define E1_IS_TMC															// C/D edited
#define E1_MAX_CURRENT 1000 												// C/D edited // in mA
#define E1_SENSE_RESISTOR 91												// C/D edited // in mOhms
#define E1_MICROSTEPS 16    												// C/D edited // number of microsteps
//#define E2_IS_TMC															// C/D edited
#define E2_MAX_CURRENT 1000 												// C/D edited // in mA
#define E2_SENSE_RESISTOR 91												// C/D edited // in mOhms
#define E2_MICROSTEPS 16    												// C/D edited // number of microsteps
//#define E3_IS_TMC															// C/D edited
#define E3_MAX_CURRENT 1000 												// C/D edited // in mA
#define E3_SENSE_RESISTOR 91												// C/D edited // in mOhms
#define E3_MICROSTEPS 16    												// C/D edited // number of microsteps 
//#define HAVE_L6470DRIVER													// C/D edited		
//#define X_IS_L6470														// C/D edited	
#define X_MICROSTEPS 16     												// C/D edited // number of microsteps
#define X_K_VAL 50          												// C/D edited // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define X_OVERCURRENT 2000  												// C/D edited // maxc current in mA. If the current goes over this value, the driver will switch off
#define X_STALLCURRENT 1500 												// C/D edited // current in mA where the driver will detect a stall
//#define X2_IS_L6470														// C/D edited 
#define X2_MICROSTEPS 16    												// C/D edited // number of microsteps
#define X2_K_VAL 50         												// C/D edited // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define X2_OVERCURRENT 2000 												// C/D edited // maxc current in mA. If the current goes over this value, the driver will switch off
#define X2_STALLCURRENT 1500 												// C/D edited // current in mA where the driver will detect a stall
//#define Y_IS_L6470                                                        // C/D edited 
#define Y_MICROSTEPS 16     												// C/D edited // number of microsteps
#define Y_K_VAL 50          												// C/D edited // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define Y_OVERCURRENT 2000  												// C/D edited // maxc current in mA. If the current goes over this value, the driver will switch off
#define Y_STALLCURRENT 1500 												// C/D edited // current in mA where the driver will detect a stall
//#define Y2_IS_L6470														// C/D edited 
#define Y2_MICROSTEPS 16    												// C/D edited // number of microsteps 
#define Y2_K_VAL 50         												// C/D edited // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define Y2_OVERCURRENT 2000 												// C/D edited // maxc current in mA. If the current goes over this value, the driver will switch off
#define Y2_STALLCURRENT 1500												// C/D edited // current in mA where the driver will detect a stall 
//#define Z_IS_L6470														// C/D edited 
#define Z_MICROSTEPS 16     												// C/D edited // number of microsteps
#define Z_K_VAL 50          												// C/D edited // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define Z_OVERCURRENT 2000  												// C/D edited // maxc current in mA. If the current goes over this value, the driver will switch off
#define Z_STALLCURRENT 1500 												// C/D edited // current in mA where the driver will detect a stall
//#define Z2_IS_L6470														// C/D edited 
#define Z2_MICROSTEPS 16    												// C/D edited // number of microsteps
#define Z2_K_VAL 50         												// C/D edited // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define Z2_OVERCURRENT 2000 												// C/D edited // maxc current in mA. If the current goes over this value, the driver will switch off
#define Z2_STALLCURRENT 1500 												// C/D edited // current in mA where the driver will detect a stall
//#define E0_IS_L6470														// C/D edited 
#define E0_MICROSTEPS 16     												// C/D edited // number of microsteps
#define E0_K_VAL 50          												// C/D edited // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define E0_OVERCURRENT 2000  												// C/D edited // maxc current in mA. If the current goes over this value, the driver will switch off
#define E0_STALLCURRENT 1500 												// C/D edited // current in mA where the driver will detect a stall
//#define E1_IS_L6470														// C/D edited 
#define E1_MICROSTEPS 16     												// C/D edited // number of microsteps
#define E1_K_VAL 50          												// C/D edited // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define E1_OVERCURRENT 2000  												// C/D edited // maxc current in mA. If the current goes over this value, the driver will switch off
#define E1_STALLCURRENT 1500 												// C/D edited // current in mA where the driver will detect a stall
//#define E2_IS_L6470														// C/D edited 
#define E2_MICROSTEPS 16     												// C/D edited // number of microsteps
#define E2_K_VAL 50          												// C/D edited // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define E2_OVERCURRENT 2000  												// C/D edited // maxc current in mA. If the current goes over this value, the driver will switch off
#define E2_STALLCURRENT 1500 												// C/D edited // current in mA where the driver will detect a stall
//#define E3_IS_L6470														// C/D edited 
#define E3_MICROSTEPS 16     												// C/D edited // number of microsteps
#define E3_K_VAL 50          												// C/D edited // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define E3_OVERCURRENT 2000  												// C/D edited // maxc current in mA. If the current goes over this value, the driver will switch off
#define E3_STALLCURRENT 1500 												// C/D edited // current in mA where the driver will detect a stall
#define BLOCK_BUFFER_SIZE 16 												// C/D edited // maximize block buffer
#define MAX_CMD_SIZE  96                                                    // C/D edited 
#define BUFSIZE        4                                                    // C/D edited 
#define NUM_POSITON_SLOTS 2													// C/D edited 
#define DROP_SEGMENTS 5                                                     // C/D edited
#define DEFAULT_MINSEGMENTTIME  20000                                       // C/D edited 
#define ARC_SUPPORT  														// C/D edited // Disabling this saves ~2738 bytes
#define MM_PER_ARC_SEGMENT 1                                                // C/D edited
#define N_ARC_CORRECTION 25													// C/D edited
#define MIN_SEGMENTS_FOR_MOVE 6                                             // C/D edited
//#define M100_FREE_MEMORY_WATCHER    										// C/D edited // Uncomment to add the M100 Free Memory Watcher for debug purpose
#define M100_FREE_MEMORY_DUMPER       										// C/D edited // Comment out to remove Dump sub-command
#define M100_FREE_MEMORY_CORRUPTOR    										// C/D edited // Comment out to remove Corrupt sub-command
//#define INCH_MODE_SUPPORT                                                 // C/D edited
//#define JSON_OUTPUT                                                       // C/D edited
//#define USE_WATCHDOG                                                      // C/D edited
//#define WATCHDOG_RESET_MANUAL												// C/D edited
//#define START_GCODE                                                       // C/D edited
#define START_PRINTING_SCRIPT "G28\nG1 Z10 F8000"                           // C/D edited
//#define STOP_GCODE                                                        // C/D edited
#define STOP_PRINTING_SCRIPT "G28\nM107\nM104 T0 S0\nM140 S0\nM84\nM81"		// C/D edited

/*****************************
 * Configuration_Temperature *
 *****************************/
#define PLA_PREHEAT_HOTEND_TEMP 210   										// C/D edited // Preheat Constants
#define PLA_PREHEAT_HPB_TEMP 50   											// C/D edited // Preheat Constants
