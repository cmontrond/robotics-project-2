package main

type Pid struct {
	Kp            float64
	Ki            float64
	Kd            float64
	Derivator     float64
	Integrator    float64
	IntegratorMax float64
	IntegratorMin float64
	SetPoint      float64
	Error         float64
	Pvalue        float64
	Dvalue        float64
	Ivalue        float64
}

func (pid *Pid) update(currentValue float64) float64 {

	pid.Error = pid.SetPoint - currentValue

	pid.Pvalue = pid.Kp * pid.Error
	pid.Dvalue = pid.Kd * (pid.Error - pid.Derivator)
	pid.Derivator = pid.Error

	pid.Integrator = pid.Integrator + pid.Error

	if pid.Integrator > pid.IntegratorMax {
		pid.Integrator = pid.IntegratorMax
	} else if pid.Integrator < pid.IntegratorMin {
		pid.Integrator = pid.IntegratorMin
	}

	pid.Ivalue = pid.Integrator * pid.Ki

	PID := pid.Pvalue + pid.Ivalue + pid.Dvalue

	return PID
}
