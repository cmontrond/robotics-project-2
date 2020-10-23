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
	SPEED       = 80
	TOO_CLOSE   = 30
	TOO_FAR     = 90
	SENSOR_SIZE = 4 // We use to discard extra cm in our final length calculation
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

func workingCode(gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver) {
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
	//	SetPoint: 60,
	//}

	// width: 31cm // 39cm
	// length: 41cm // 57cm

	// third box: 43.5 / 40

	pid := NewPID(1, 1.0, 1.0, 0.0)
	err := pid.SetTunings(1.0, 1.0, 0.0)
	err = pid.SetOutputLimits(-1000.0, 1000.0)
	err = pid.SetSampleTime(1) // sample time in seconds

	err = lidarSensor.Start()
	if err != nil {
		fmt.Println("error starting lidarSensor")
	}

	for {
		lidarReading, err := lidarSensor.Distance()

		if err != nil {
			fmt.Println("Error reading lidar sensor %+v", err)
		}

		pidOutput := pid.Compute(20.0, float64(lidarReading))
		fmt.Printf("PID OUTPUT: %.2f\n", pidOutput)
	}

	//firstSideStart := false
	//firstSideFinished := false
	//firstTurnFinished := false
	//secondSideStart := false
	//secondSideFinished := false
	//secondTurnFinished := false
	//thirdSideStart := false
	//thirdSideFinished := false
	//thirdTurnFinished := false
	//fourthSideStart := false
	//fourthSideFinished := false
	//
	//firstSideStartEncodersVal := 0.0
	//secondSideStartEncodersVal := 0.0
	//thirdSideStartEncodersVal := 0.0
	//fourthSideStartEncodersVal := 0.0
	//
	//firstSideLength := 0.0
	//secondSideLength := 0.0
	//thirdSideLength := 0.0
	//fourthSideLength := 0.0
	//
	//// TODO: maybe have a turning boolean just so you don't do pid when turning. Maight not be necessary since you're doing turning in one bloc
	//
	//pidEnabled := false
	//pidOutput := 0.0
	////debug := false
	//
	//err = lidarSensor.Start()
	//if err != nil {
	//	fmt.Println("error starting lidarSensor")
	//}
	//
	//encodersVal := ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
	//
	////println("Initial Encoders Value (in cm): ", encodersVal)
	//
	//for {
	//
	//	if !fourthSideFinished {
	//		lidarReading, err := lidarSensor.Distance()
	//
	//		if err != nil {
	//			fmt.Println("Error reading lidar sensor %+v", err)
	//		}
	//
	//		println("Lidar Sensor Value:", lidarReading)
	//
	//		// FIRST SIDE
	//		if lidarReading < 105 && !firstSideStart {
	//			firstSideStart = true
	//			println("FIRST SIDE: STARTED")
	//			pidEnabled = true
	//			pid.Reset()
	//			pidOutput = 0.0
	//			Stop(gopigo3)
	//			time.Sleep(time.Second)
	//			firstSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
	//		}
	//
	//		if lidarReading > 105 && firstSideStart && !firstSideFinished {
	//			firstSideFinished = true
	//			pidEnabled = false
	//			Stop(gopigo3)
	//			time.Sleep(time.Second)
	//			encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
	//			println("FIRST SIDE FINISHED")
	//			firstSideLength = math.Abs(encodersVal - firstSideStartEncodersVal)
	//		}
	//
	//		if lidarReading > 100 && firstSideFinished && !firstTurnFinished {
	//			pidEnabled = false
	//			Forward(gopigo3, -SPEED)
	//			time.Sleep(time.Second * 2)
	//			SpinRight(gopigo3, SPEED)
	//			time.Sleep(time.Millisecond * 2200)
	//			firstTurnFinished = true
	//			println("Finished First Turn")
	//		}
	//
	//		// SECOND SIDE
	//		if lidarReading < 105 && firstSideFinished && firstTurnFinished && !secondSideStart {
	//			secondSideStart = true
	//			pid.Reset()
	//			pidOutput = 0.0
	//			//debug = true
	//			println("SECOND SIDE: STARTED")
	//			pidEnabled = true
	//			Stop(gopigo3)
	//			time.Sleep(time.Second)
	//			secondSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
	//		}
	//
	//		if lidarReading > 105 && secondSideStart && !secondSideFinished {
	//			secondSideFinished = true
	//			pidEnabled = false
	//			Stop(gopigo3)
	//			time.Sleep(time.Second)
	//			encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
	//			println("SECOND SIDE FINISHED")
	//			secondSideLength = math.Abs(encodersVal - secondSideStartEncodersVal)
	//		}
	//
	//		if lidarReading > 100 && secondSideFinished && !secondTurnFinished {
	//			pidEnabled = false
	//			Forward(gopigo3, -SPEED)
	//			time.Sleep(time.Second * 2)
	//			SpinRight(gopigo3, SPEED)
	//			time.Sleep(time.Millisecond * 2200)
	//			secondTurnFinished = true
	//			println("Finished Second Turn")
	//		}
	//
	//		// THIRD SIDE
	//		if lidarReading < 105 && secondSideFinished && secondTurnFinished && !thirdSideStart {
	//			thirdSideStart = true
	//			println("THIRD SIDE: STARTED")
	//			pidEnabled = true
	//			pid.Reset()
	//			pidOutput = 0.0
	//			Stop(gopigo3)
	//			time.Sleep(time.Second)
	//			thirdSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
	//		}
	//
	//		if lidarReading > 105 && thirdSideStart && !thirdSideFinished {
	//			thirdSideFinished = true
	//			pidEnabled = false
	//			Stop(gopigo3)
	//			time.Sleep(time.Second)
	//			encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
	//			println("THIRD SIDE FINISHED")
	//			thirdSideLength = math.Abs(encodersVal - thirdSideStartEncodersVal)
	//		}
	//
	//		if lidarReading > 100 && thirdSideFinished && !thirdTurnFinished {
	//			pidEnabled = false
	//			Forward(gopigo3, -SPEED)
	//			time.Sleep(time.Second * 2)
	//			SpinRight(gopigo3, SPEED)
	//			time.Sleep(time.Millisecond * 2200)
	//			thirdTurnFinished = true
	//			println("Finished Third Turn")
	//		}
	//
	//		// FOURTH SIDE
	//		if lidarReading < 105 && thirdSideFinished && thirdTurnFinished && !fourthSideStart {
	//			fourthSideStart = true
	//			pidEnabled = false
	//			pid.Reset()
	//			pidOutput = 0.0
	//			println("FOURTH SIDE: STARTED")
	//			pidEnabled = true
	//			Stop(gopigo3)
	//			time.Sleep(time.Second)
	//			fourthSideStartEncodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
	//		}
	//
	//		if lidarReading > 105 && fourthSideStart && !fourthSideFinished {
	//			pidEnabled = false
	//			pid.Reset()
	//			pidOutput = 0.0
	//			fourthSideFinished = true
	//			println("Finished")
	//			Stop(gopigo3)
	//			time.Sleep(time.Second)
	//			encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
	//			println("FOURTH SIDE FINISHED")
	//			fourthSideLength = math.Abs(encodersVal - fourthSideStartEncodersVal)
	//		}
	//
	//		//encodersVal = ReadEncodersAverage(gopigo3, g.WHEEL_CIRCUMFERENCE)
	//		//fmt.Printf("Current Encoders Value (in cm): %.2f\n", encodersVal)
	//
	//		// Here, should actually decide if you move a little bit to the left, right, or continue forward
	//		if pidEnabled {
	//			// This is where PID logic should go?
	//			pidOutput = pid.Compute(20.0, float64(lidarReading))
	//			fmt.Printf("PID OUTPUT: %.2f\n", pidOutput)
	//		}
	//		Forward(gopigo3, -SPEED)
	//		time.Sleep(time.Second)
	//
	//	} else {
	//		Stop(gopigo3)
	//		BlinkLED(gopigo3)
	//		println("====================================\n\n")
	//		fmt.Printf("First side length: %.2f cm\n", firstSideLength-SENSOR_SIZE)
	//		fmt.Printf("Second side length: %.2f cm\n", secondSideLength-SENSOR_SIZE)
	//		fmt.Printf("Third side length: %.2f cm\n", thirdSideLength-SENSOR_SIZE)
	//		fmt.Printf("Fourth side length: %.2f cm\n\n", fourthSideLength-SENSOR_SIZE)
	//		fmt.Printf("Difference between first and third sides: %.2f cm\n", math.Abs(firstSideLength-thirdSideLength))
	//		fmt.Printf("Difference between second and fourth sides: %.2f cm\n\n", math.Abs(secondSideLength-fourthSideLength))
	//		println("====================================\n\n")
	//	}
	//}
}

func robotRunLoop(gopigo3 *g.Driver, lidarSensor *i2c.LIDARLiteDriver) {
	workingCode(gopigo3, lidarSensor)

	//for {
	//	lidarReading, err := lidarSensor.Distance()
	//
	//	if err != nil {
	//		fmt.Println("Error reading lidar sensor %+v", err)
	//	}
	//
	//	println("Lidar Sensor Value:", lidarReading)
	//
	//	time.Sleep(time.Second)
	//}
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
