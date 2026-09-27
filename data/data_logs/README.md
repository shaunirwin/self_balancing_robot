# README

This is the readme file for the data recordings in this folder. It describes what each was recorded for and when.

`balance-recording_11Hz_oscillation.bin`
* Created 2026-09-24
* Description: Recording of balancing attempt, where the robot exhibited a very pronounced ~11Hz oscillation on PID output, gyro, pitch, PWM. Robot falls over after a few seconds.
* Useful for: Debugging why the robot is not yet balancing correctly.


`balance-recording_gyros_noisy_motors_on.sbrpb`
* Created 2026-09-26
* Description: In this case the motors were on and the gyros were very noisy.
* Useful for: Investigating noisiness in the gyros. 

`balance-recording_gyros_smooth_motors_off.sbrpb`
* Created 2026-09-26
* Description: In this case the motors were off and the gyros were very smooth.
* Useful for: Investigating noisiness in the gyros.

`20260927-003031_recording_manual_step_motor1_to_35pwm.sbrpb`
* Description: Manual mode. Step input from zero pwm to 35. Gyro readings noisy despite feedback loop being off.
* Useful for: Investigating noisiness in the gyros.

`20260927-003253_recording_manual_step_motor1_to_150pwm.sbrpb`
* Description: Manual mode. Step input from zero pwm to 150. Gyro readings noisy despite feedback loop being off.
* Useful for: Investigating noisiness in the gyros.

`20260927-005004_recording_manual_step_motor2_to_35pwm.sbrpb`
* Description: Manual mode. Step input from zero pwm to 35. Gyro readings noisy despite feedback loop being off.
* Useful for: Investigating noisiness in the gyros.

`20260927-005226_recording_manual_step_motor2_to_150pwm.sbrpb`
* Description: Manual mode. Step input from zero pwm to 150. Gyro readings noisy despite feedback loop being off.
* Useful for: Investigating noisiness in the gyros.

`20260927-010502_recording_11hz_oscillation_auto_mode_150hz_sampling.sbrpb`
* Description: Recreated the same experiment as recorded in `balance-recording_11Hz_oscillation.bin`, which was previously done with 100Hz sampling rate. Still same 11Hz oscillation seen when using the 150Hz sampling rate.
* Useful for: Debugging balancing
