/*
 * protocol.h
 *
 * Created on: 2014. 4. 12.
 * Author: Seokyong Hong
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#define PROTOCOL_OPEN_CHARACTER		0x01
#define PROTOCOL_CLOSE_CHARACTER	0x04

#define PROTOCOL_TYPE_MOTOR			'M'
#define PROTOCOL_TYPE_SENSOR		'S'

#define PROTOCOL_MOTOR_FORWARD		'F'
#define PROTOCOL_MOTOR_BACKWARD		'B'
#define PROTOCOL_MOTOR_TURN_LEFT	'L'
#define PROTOCOL_MOTOR_TURN_RIGHT	'R'
#define PROTOCOL_MOTOR_STOP			'S'
#define PROTOCOL_SENSOR_ULTRASONIC	'U'

#endif /* PROTOCOL_H_ */
