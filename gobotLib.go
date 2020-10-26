package main

import (
	"fmt"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"time"
)

func Forward(gopigo3 *g.Driver, speed int) {
	err := gopigo3.SetMotorDps(g.MOTOR_LEFT+g.MOTOR_RIGHT, speed)
	if err != nil {
		fmt.Errorf("Error moving the robot forward: %+v", err)
	}
}

func Left(gopigo3 *g.Driver, speed int) {
	err := gopigo3.SetMotorDps(g.MOTOR_LEFT, 0)
	if err != nil {
		fmt.Errorf("Error moving the robot left: %+v", err)
	}
	err = gopigo3.SetMotorDps(g.MOTOR_RIGHT, speed)
	if err != nil {
		fmt.Errorf("Error moving the robot left: %+v", err)
	}
}

func Right(gopigo3 *g.Driver, speed int) {
	err := gopigo3.SetMotorDps(g.MOTOR_LEFT, speed)
	if err != nil {
		fmt.Errorf("Error moving the robot right: %+v", err)
	}
	err = gopigo3.SetMotorDps(g.MOTOR_RIGHT, 0)
	if err != nil {
		fmt.Errorf("Error moving the robot right: %+v", err)
	}
}

func SpinRight(gopigo3 *g.Driver, speed int) {
	err := gopigo3.SetMotorDps(g.MOTOR_LEFT, speed)
	if err != nil {
		fmt.Errorf("Error doing spin right: %+v", err)
	}
	err = gopigo3.SetMotorDps(g.MOTOR_RIGHT, speed*-1)
	if err != nil {
		fmt.Errorf("Error doing spin right: %+v", err)
	}
}

func ReadEnconders(gopigo3 *g.Driver) (int64, int64) {
	leftEncoder, err := gopigo3.GetMotorEncoder(g.MOTOR_LEFT)

	if err != nil {
		fmt.Errorf("Error reading left encoder: %+v", err)
	}

	rightEncoder, err := gopigo3.GetMotorEncoder(g.MOTOR_RIGHT)

	if err != nil {
		fmt.Errorf("Error reading right encoder: %+v", err)
	}

	return leftEncoder, rightEncoder
}

func ReadEncodersAverage(gopigo3 *g.Driver, wheelCircumference float64) float64 {

	left, right := ReadEnconders(gopigo3)

	average := float64((left + right) / 2)

	// We want the return value in centimetres, so that's why we do this
	average = ((average / 360) * wheelCircumference) / 10

	return average
}

func BlinkLED(gopigo3 *g.Driver) {
	err := gopigo3.SetLED(g.LED_EYE_RIGHT, 0x00, 0x00, 0xFF)
	if err != nil {
		fmt.Println(err)
	}

	time.Sleep(time.Second)

	err = gopigo3.SetLED(g.LED_EYE_RIGHT, 0x00, 0x00, 0x00)
	if err != nil {
		fmt.Println(err)
	}

	time.Sleep(time.Second)
}

func Stop(gopigo3 *g.Driver) {
	err := gopigo3.SetMotorDps(g.MOTOR_LEFT+g.MOTOR_RIGHT, 0)
	if err != nil {
		fmt.Errorf("Error stopping the robot: %+v", err)
	}
}
