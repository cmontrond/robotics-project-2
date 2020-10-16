package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/i2c"
	g "gobot.io/x/gobot/platforms/dexter/gopigo3"
	"gobot.io/x/gobot/platforms/raspi"
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

	average = ((average / 360) * wheelCircumference) / 10

	return average
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

	err := lidarSensor.Start()
	if err != nil {
		fmt.Println("error starting lidarSensor")
	}

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
			}

			if lidarReading > 105 && firstSideStart && !firstSideFinished {
				firstSideFinished = true
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
			}

			if lidarReading > 105 && secondSideStart && !secondSideFinished {
				secondSideFinished = true
			}

			if lidarReading > 100 && secondSideFinished && !secondTurnFinished {
				Forward(gopigo3, -SPEED)
				time.Sleep(time.Second * 2)
				SpinRight(gopigo3, SPEED)
				time.Sleep(time.Millisecond * 2200)
				secondTurnFinished = true
				println("Finished First Turn")
			}

			// THIRD SIDE
			if lidarReading < 105 && secondSideFinished && secondTurnFinished {
				thirdSideStart = true
			}

			if lidarReading > 105 && thirdSideStart && !thirdSideFinished {
				thirdSideFinished = true
			}

			if lidarReading > 100 && thirdSideFinished && !thirdTurnFinished {
				Forward(gopigo3, -SPEED)
				time.Sleep(time.Second * 2)
				SpinRight(gopigo3, SPEED)
				time.Sleep(time.Millisecond * 2200)
				thirdTurnFinished = true
				println("Finished First Turn")
			}

			// FOURTH SIDE
			if lidarReading < 105 && thirdSideFinished && thirdTurnFinished {
				fourthSideStart = true
			}

			if lidarReading > 105 && fourthSideStart && !fourthSideFinished {
				fourthSideFinished = true
				println("Finished")
			}

			Forward(gopigo3, -SPEED)
			time.Sleep(time.Second)
		}

		Stop(gopigo3)
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
