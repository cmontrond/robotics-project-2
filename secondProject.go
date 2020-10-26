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
	SPEED       = 40
	SENSOR_SIZE = 4 // We use to discard extra cm in our final length calculation
)

func robotRunLoop(gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver) {

	pid := NewPID(1, 1.0, 1.0, 0.0)
	err := pid.SetTunings(1.0, 1.0, 0.0)
	err = pid.SetOutputLimits(-450.0, 450.0)
	err = pid.SetSampleTime(1) // sample time in seconds

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

	pidEnabled := false
	pidOutput := 0.0

	err = lidarSensor.Start()
	if err != nil {
		fmt.Println("error starting lidarSensor")
	}

	encodersVal := ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)

	for {

		if !fourthSideFinished {
			lidarReading, err := lidarSensor.Distance()

			if err != nil {
				fmt.Println("Error reading lidar sensor %+v", err)
			}

			println("Lidar Sensor Value:", lidarReading)

			// FIRST SIDE
			if lidarReading < 50 && !firstSideStart {
				firstSideStart = true
				println("FIRST SIDE: STARTED")
				pidEnabled = true
				pid.Reset()
				pidOutput = 0.0
				Stop(gopigo3)
				time.Sleep(time.Second)
				firstSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
			}

			if lidarReading > 50 && firstSideStart && !firstSideFinished {
				firstSideFinished = true
				pidEnabled = false
				Stop(gopigo3)
				time.Sleep(time.Second)
				encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
				println("FIRST SIDE FINISHED")
				firstSideLength = math.Abs(encodersVal - firstSideStartEncodersVal)
			}

			if lidarReading > 45 && firstSideFinished && !firstTurnFinished {
				pidEnabled = false
				Forward(gopigo3, -SPEED)
				time.Sleep(time.Second * 4)
				SpinRight(gopigo3, SPEED)
				time.Sleep(time.Millisecond * 4400)
				firstTurnFinished = true
				println("Finished First Turn")
			}

			// SECOND SIDE
			if lidarReading < 50 && firstSideFinished && firstTurnFinished && !secondSideStart {
				secondSideStart = true
				pid.Reset()
				pidOutput = 0.0
				println("SECOND SIDE: STARTED")
				pidEnabled = true
				Stop(gopigo3)
				time.Sleep(time.Second)
				secondSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
			}

			if lidarReading > 50 && secondSideStart && !secondSideFinished {
				secondSideFinished = true
				pidEnabled = false
				Stop(gopigo3)
				time.Sleep(time.Second)
				encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
				println("SECOND SIDE FINISHED")
				secondSideLength = math.Abs(encodersVal - secondSideStartEncodersVal)
			}

			if lidarReading > 45 && secondSideFinished && !secondTurnFinished {
				pidEnabled = false
				Forward(gopigo3, -SPEED)
				time.Sleep(time.Second * 4)
				SpinRight(gopigo3, SPEED)
				time.Sleep(time.Millisecond * 4400)
				secondTurnFinished = true
				println("Finished Second Turn")
			}

			// THIRD SIDE
			if lidarReading < 50 && secondSideFinished && secondTurnFinished && !thirdSideStart {
				thirdSideStart = true
				println("THIRD SIDE: STARTED")
				pidEnabled = true
				pid.Reset()
				pidOutput = 0.0
				Stop(gopigo3)
				time.Sleep(time.Second)
				thirdSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
			}

			if lidarReading > 50 && thirdSideStart && !thirdSideFinished {
				thirdSideFinished = true
				pidEnabled = false
				Stop(gopigo3)
				time.Sleep(time.Second)
				encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
				println("THIRD SIDE FINISHED")
				thirdSideLength = math.Abs(encodersVal - thirdSideStartEncodersVal)
			}

			if lidarReading > 45 && thirdSideFinished && !thirdTurnFinished {
				pidEnabled = false
				Forward(gopigo3, -SPEED)
				time.Sleep(time.Second * 4)
				SpinRight(gopigo3, SPEED)
				time.Sleep(time.Millisecond * 4400)
				thirdTurnFinished = true
				println("Finished Third Turn")
			}

			// FOURTH SIDE
			if lidarReading < 50 && thirdSideFinished && thirdTurnFinished && !fourthSideStart {
				fourthSideStart = true
				pidEnabled = false
				pid.Reset()
				pidOutput = 0.0
				println("FOURTH SIDE: STARTED")
				pidEnabled = true
				Stop(gopigo3)
				time.Sleep(time.Second)
				fourthSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
			}

			if lidarReading > 50 && fourthSideStart && !fourthSideFinished {
				pidEnabled = false
				pid.Reset()
				pidOutput = 0.0
				fourthSideFinished = true
				println("Finished")
				Stop(gopigo3)
				time.Sleep(time.Second)
				encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
				println("FOURTH SIDE FINISHED")
				fourthSideLength = math.Abs(encodersVal - fourthSideStartEncodersVal)
			}

			if pidEnabled {
				pidOutput = pid.Compute(20.0, float64(lidarReading))
				fmt.Printf("PID OUTPUT: %.2f\n", pidOutput)
				if pidOutput > 30 {
					Right(gopigo3, -SPEED)
					time.Sleep(time.Millisecond * 500)
					Forward(gopigo3, -SPEED)
					time.Sleep(time.Millisecond * 500)
					Left(gopigo3, -SPEED)
					time.Sleep(time.Millisecond * 500)
					Forward(gopigo3, -SPEED)
					time.Sleep(time.Millisecond * 500)

				} else if pidOutput < 5 {
					Left(gopigo3, -SPEED)
					time.Sleep(time.Millisecond * 500)
					Forward(gopigo3, -SPEED)
					time.Sleep(time.Millisecond * 500)
					Right(gopigo3, -SPEED)
					time.Sleep(time.Millisecond * 500)
					Forward(gopigo3, -SPEED)
					time.Sleep(time.Millisecond * 500)
				} else {
					Forward(gopigo3, -SPEED)
					time.Sleep(time.Second)
				}
			} else {
				Forward(gopigo3, -SPEED)
				time.Sleep(time.Second)
			}

		} else {
			Stop(gopigo3)
			BlinkLED(gopigo3)
			println("====================================\n\n")
			fmt.Printf("First side length: %.2f cm\n", firstSideLength-SENSOR_SIZE)
			fmt.Printf("Second side length: %.2f cm\n", secondSideLength-SENSOR_SIZE)
			fmt.Printf("Third side length: %.2f cm\n", thirdSideLength-SENSOR_SIZE)
			fmt.Printf("Fourth side length: %.2f cm\n\n", fourthSideLength-SENSOR_SIZE)
			fmt.Printf("Difference between first and third sides: %.2f cm\n", math.Abs(firstSideLength-thirdSideLength))
			fmt.Printf("Difference between second and fourth sides: %.2f cm\n\n", math.Abs(secondSideLength-fourthSideLength))
			println("====================================\n\n")
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

	robot := gobot.NewRobot("Project 2: Block Measurer",
		[]gobot.Connection{raspiAdaptor},
		[]gobot.Device{gopigo3, lidarSensor},
		mainRobotFunc,
	)

	err := robot.Start()

	if err != nil {
		fmt.Errorf("Error starting the Robot %+v", err)
	}
}
