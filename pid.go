package main

import (
	"errors"
)

type PID struct {
	iTerm, lastInput, minOutput, maxOutput, kp, ki, kd float64
	sampleTime                                         int
	limitsActive                                       bool
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
	pid.iTerm = pid.ki * err
	if pid.limitsActive {
		if pid.iTerm > pid.maxOutput {
			pid.iTerm = pid.maxOutput
		} else if pid.iTerm < pid.minOutput {
			pid.iTerm = pid.minOutput
		}
	}
	dInput := input - pid.lastInput
	output := pid.kp*err + pid.ki*pid.iTerm + pid.kd*dInput
	if pid.limitsActive {
		if output > pid.maxOutput {
			output = pid.maxOutput
		} else if output < pid.minOutput {
			output = pid.minOutput
		}
	}
	pid.lastInput = 0.0
	return output
}

func (pid *PID) Reset() {
	pid.iTerm = 0.0
	pid.lastInput = 0
}
