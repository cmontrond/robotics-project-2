package main

import (
	"errors"
)

type PID struct {
	iTerm, lastErr, minOutput, maxOutput, kp, ki, kd float64
	sampleTime                                       int
	limitsActive                                     bool
}

func NewPID(interval int, kp, ki, kd float64) *PID {
	return &PID{
		sampleTime: interval,
		kp:         kp,
		ki:         ki,
		kd:         kd,
	}
}

func (pid *PID) SetOutputLimits(min, max float64) error {
	if min >= max {
		return errors.New("minimum should be smaller than maximum")
	}
	pid.limitsActive = true
	pid.minOutput = min
	pid.maxOutput = max
	return nil
}

func (pid *PID) SetTunings(kp, ki, kd float64) error {
	if kp < 0 || ki < 0 || kd < 0 {
		return errors.New("tuning parameters must be greater than 0")
	}
	pid.kp = kp
	pid.ki = ki * float64(pid.sampleTime)
	pid.kd = ki / float64(pid.sampleTime)
	return nil
}

func (pid *PID) SetSampleTime(seconds int) error {
	if seconds < 1 {
		return errors.New("the interval must be > 0 seconds")
	}
	var ratio float64
	if pid.sampleTime > 0 {
		ratio = float64(seconds) / float64(pid.sampleTime)
	} else {
		ratio = 1
	}
	pid.ki = pid.ki * ratio
	pid.kd = pid.kd / ratio
	pid.sampleTime = seconds
	return nil
}

func (pid *PID) Compute(setPoint, input float64) float64 {
	err := setPoint - input
	pid.iTerm += pid.ki * err
	if pid.limitsActive {
		if pid.iTerm > pid.maxOutput {
			pid.iTerm = pid.maxOutput
		} else if pid.iTerm < pid.minOutput {
			pid.iTerm = pid.minOutput
		}
	}
	dInput := input - pid.lastErr
	output := pid.kp*err + pid.ki*pid.iTerm + pid.kd*dInput
	if pid.limitsActive {
		if output > pid.maxOutput {
			output = pid.maxOutput
		} else if output < pid.minOutput {
			output = pid.minOutput
		}
	}
	pid.lastErr = err
	return output
}

//func main() {
//	inputs := []float64{34.0, 100.0, 600.0, 20.0, 90.0, 60.0}
//	setPoint := 60.0
//	pid := NewPID(1, 1.0, 1.0, 0.0)
//	err := pid.SetSampleTime(1)
//
//	if err != nil {
//		println("Error")
//	}
//
//	for i, value := range inputs {
//		println("Index: ", i)
//		fmt.Printf("SetPoint: %.2f\n", setPoint)
//		fmt.Printf("Input: %.2f\n", value)
//		fmt.Printf("LastInput: %.2f\n", pid.lastErr)
//		fmt.Printf("Output: %.2f\n", pid.Compute(setPoint, value))
//		fmt.Printf("\n\n")
//	}
//}
