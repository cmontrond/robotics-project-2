package main

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/aio"
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

func Stop(gopigo3 *g.Driver) {
	err := gopigo3.SetMotorDps(g.MOTOR_LEFT+g.MOTOR_RIGHT, 0)
	if err != nil {
		fmt.Errorf("Error stopping the robot: %+v", err)
	}
}

func robotRunLoop(gopigo3 *g.Driver, leftLightSensor *aio.GroveLightSensorDriver, rightLightSensor *aio.GroveLightSensorDriver, lidarSensor *i2c.LIDARLiteDriver) {

	// We know that when it's under 100, it's close enough
	// You will need to use the wheel size to get

	//firstSideStart := false
	//firstSideFinished := false
	//finished := false

	//err := lidarSensor.Start()
	//if err != nil {
	//	fmt.Println("error starting lidarSensor")
	//}

	for {

		time.Sleep(time.Second)
		SpinRight(gopigo3, SPEED)
		time.Sleep(time.Second * (3 / 2))
		Forward(gopigo3, -SPEED)
		time.Sleep(time.Second * 3)

		//if !finished {
		//	lidarReading, err := lidarSensor.Distance()
		//
		//	if err != nil {
		//		fmt.Println("Error reading lidar sensor %+v", err)
		//	}
		//
		//	println("Lidar Sensor Value:", lidarReading)
		//
		//	if lidarReading < 100 && !firstSideStart {
		//		firstSideStart = true
		//	}
		//
		//	if lidarReading > 100 && firstSideStart && !firstSideFinished {
		//		firstSideFinished = true
		//	}
		//
		//	if lidarReading > 100 && firstSideStart && firstSideFinished {
		//
		//	}
		//
		//	Forward(gopigo3, -SPEED)
		//
		//	time.Sleep(time.Second)
		//}
		//
		//Stop(gopigo3)
	}
}

func main() {

	raspiAdaptor := raspi.NewAdaptor()
	gopigo3 := g.NewDriver(raspiAdaptor)

	leftLightSensor := aio.NewGroveLightSensorDriver(gopigo3, "AD_2_1")
	rightLightSensor := aio.NewGroveLightSensorDriver(gopigo3, "AD_1_1")
	lidarSensor := i2c.NewLIDARLiteDriver(raspiAdaptor)

	mainRobotFunc := func() {
		robotRunLoop(gopigo3, leftLightSensor, rightLightSensor, lidarSensor)
	}

	robot := gobot.NewRobot("Project 2",
		[]gobot.Connection{raspiAdaptor},
		[]gobot.Device{gopigo3, leftLightSensor, rightLightSensor, lidarSensor},
		mainRobotFunc,
	)

	err := robot.Start()

	if err != nil {
		fmt.Errorf("Error starting the Robot %+v", err)
	}
}
