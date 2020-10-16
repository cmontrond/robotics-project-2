package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/i2c"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
	"math"
	"time"
)

const (
	SPEED = 80
)

func Forward(gopigo3 *g.Driver, speed int) {
	err := gopigo3.SetMotorDps(g.MOTOR_LEFT+g.MOTOR_RIGHT, speed)
	if err != nil {
		fmt.Errorf("Error moving the robot forward: %+v", err)
	}
}

func SpinRight(gopigo3 *g.Driver, speed int) {
	err := gopigo3.SetMotorDps(g.MOTOR_LEFT, speed)
	if err != nil {
		fmt.Errorf("Error moving the robot backward: %+v", err)
	}
	err = gopigo3.SetMotorDps(g.MOTOR_RIGHT, speed*-1)
	if err != nil {
		fmt.Errorf("Error moving the robot backward: %+v", err)
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

func robotRunLoop(gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver) {

	// We know that when it's under 105, it's close enough
	// You will need to use the wheel size to get

	// Proportional-Integral-Derivative (PID) controller
	//pid := Pid {
	//	Kp:            2.0,
	//	Ki:            0.0,
	//	Kd:            1.0,
	//	Derivator:     0,
	//	Integrator:    0,
	//	IntegratorMax: 500,
	//	IntegratorMin: 500,
	//}

	firstSideStart := false
	firstSideFinished := false
	firstTurnFinished := false
	secondSideStart := false
	secondSideFinished := false
	secondTurnFinished := false
	thirdSideStart := false
	thirdSideFinished := false
	thirdTurnFinished := false
	fourthSideStart := false
	fourthSideFinished := false

	firstSideStartEncodersVal := 0.0
	secondSideStartEncodersVal := 0.0
	thirdSideStartEncodersVal := 0.0
	fourthSideStartEncodersVal := 0.0

	firstSideLength := 0.0
	secondSideLength := 0.0
	thirdSideLength := 0.0
	fourthSideLength := 0.0

	err := lidarSensor.Start()
	if err != nil {
		fmt.Println("error starting lidarSensor")
	}

	encodersVal := ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)

	println("Initial Encoders Value (in cm): ", encodersVal)

	for {

		if !fourthSideFinished {
			lidarReading, err := lidarSensor.Distance()

			if err != nil {
				fmt.Println("Error reading lidar sensor %+v", err)
			}

			println("Lidar Sensor Value:", lidarReading)

			// FIRST SIDE
			if lidarReading < 105 && !firstSideStart {
				firstSideStart = true
				firstSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
			}

			if lidarReading > 105 && firstSideStart && !firstSideFinished {
				firstSideFinished = true
				encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
				fmt.Printf("FIRST SIDE FINISHED: Encoders Value (in cm): %f\n", encodersVal)
				firstSideLength = math.Abs(encodersVal - firstSideStartEncodersVal)
			}

			if lidarReading > 100 && firstSideFinished && !firstTurnFinished {
				Forward(gopigo3, -SPEED)
				time.Sleep(time.Second * 2)
				SpinRight(gopigo3, SPEED)
				time.Sleep(time.Millisecond * 2200)
				firstTurnFinished = true
				println("Finished First Turn")
			}

			// SECOND SIDE
			if lidarReading < 105 && firstSideFinished && firstTurnFinished {
				secondSideStart = true
				secondSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
			}

			if lidarReading > 105 && secondSideStart && !secondSideFinished {
				secondSideFinished = true
				encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
				fmt.Printf("SECOND SIDE FINISHED: Encoders Value (in cm): %f\n", encodersVal)
				secondSideLength = math.Abs(encodersVal - secondSideStartEncodersVal)
			}

			if lidarReading > 100 && secondSideFinished && !secondTurnFinished {
				Forward(gopigo3, -SPEED)
				time.Sleep(time.Second * 2)
				SpinRight(gopigo3, SPEED)
				time.Sleep(time.Millisecond * 2200)
				secondTurnFinished = true
				println("Finished Second Turn")
			}

			// THIRD SIDE
			if lidarReading < 105 && secondSideFinished && secondTurnFinished {
				thirdSideStart = true
				thirdSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
			}

			if lidarReading > 105 && thirdSideStart && !thirdSideFinished {
				thirdSideFinished = true
				encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
				fmt.Printf("THIRD SIDE FINISHED: Encoders Value (in cm): %f\n", encodersVal)
				thirdSideLength = math.Abs(encodersVal - thirdSideStartEncodersVal)
			}

			if lidarReading > 100 && thirdSideFinished && !thirdTurnFinished {
				Forward(gopigo3, -SPEED)
				time.Sleep(time.Second * 2)
				SpinRight(gopigo3, SPEED)
				time.Sleep(time.Millisecond * 2200)
				thirdTurnFinished = true
				println("Finished Third Turn")
			}

			// FOURTH SIDE
			if lidarReading < 105 && thirdSideFinished && thirdTurnFinished {
				fourthSideStart = true
				fourthSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
			}

			if lidarReading > 105 && fourthSideStart && !fourthSideFinished {
				fourthSideFinished = true
				println("Finished")
				encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
				fmt.Printf("FOURTH SIDE FINISHED: Encoders Value (in cm): %f\n", encodersVal)
				fourthSideLength = math.Abs(encodersVal - fourthSideStartEncodersVal)
			}

			encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
			fmt.Printf("Current Encoders Value (in cm): %f\n", encodersVal)

			Forward(gopigo3, -SPEED)
			time.Sleep(time.Second)

		} else {
			Stop(gopigo3)
			BlinkLED(gopigo3)
			fmt.Printf("First side length (in cm): %f", firstSideLength)
			fmt.Printf("Second side length (in cm): %f", secondSideLength)
			fmt.Printf("Third side length (in cm): %f", thirdSideLength)
			fmt.Printf("Fourth side length (in cm): %f", fourthSideLength)
		}
	}
}

func main() {

	raspiAdaptor := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspiAdaptor)

	lidarSensor := i2c.NewLIDARLiteDriver(raspiAdaptor)

	mainRobotFunc := func() {
		robotRunLoop(gopigo3, lidarSensor)
	}

	robot := gobot.NewRobot("Project 2",
		[]gobot.Connection{raspiAdaptor},
		[]gobot.Device{gopigo3, lidarSensor},
		mainRobotFunc,
	)

	err := robot.Start()

	if err != nil {
		fmt.Errorf("Error starting the Robot %+v", err)
	}
}
