# Robotics Project 2

Second Project for COMP 570: Robotics at Bridgewater State University.<br>

This project aims to create a robot, and a program that that measures a box.
By doing this project, the student will continue gaining experience doing sensor/effector fusion, using sensor data 
to drive behavior, using new sensors and control mechanisms.

## Details

* The robot will start near an object in the world (A box) 
* The robot needs to travel along the box using some form of feedback control (not proportionalbut PI, PD, or PID
* The robot needs to measure the box as it goes using the encoders on the motors (there are constants in 
the gopigo3 package for the wheel circumference.)
* The robot needs to go all the way around the box, get both length readings, both width 
readings and report length, width, and the measurement error between the two readings in each set

## Getting Started

These instructions will get you a copy of the project up and running on your local robot for development 
and testing purposes.

### Prerequisites

* GoPiGo3 Kit;
* 1 Lidar Sensor;
* Have Go Installed;

### Running the Project

Git Clone this repository:

```
git clone https://github.com/cmontrond/robotics-project-2
```

CD into the project folder:

```
cd robotics-project-2
```

Compile the source code:

```
go build secondProject.go
```

Run the project:

```
./secondProject
```

## Built With

* [GoPiGo3](https://www.dexterindustries.com/gopigo3/) - The Robotics Kit
* [LIDAR-Lite](https://www.sparkfun.com/products/14032) - The LIDAR Sensor
* [Go](https://golang.org//) - The Programming Language
* [Gobot](https://gobot.io/) - The Robotics/IoT Framework

## Author

**Christopher Montrond da Veiga Fernandes** - [Contact](mailto:cmontronddaveigafern@student.bridgew.edu)

## Instructor

**Dr. John Santore** - [Contact](mailto:jsantore@bridgew.edu)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
