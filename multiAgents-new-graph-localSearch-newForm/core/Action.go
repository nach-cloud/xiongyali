package core

type Action struct {
	TaskID        int
	ChargePointId int
	Time          float64 //完成任务需要多久时间
	StartTime     int     //开始时刻
	Date          int
	CostPow       float64 //消耗电量
	Car           *Car
	Drone         *Drone
	Worker        *Worker
	ActionType    NodeType
	Destination   Position
}
