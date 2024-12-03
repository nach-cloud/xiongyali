package core

import (
	"sync"
)

// 任务点 结构体
type TaskPoint struct {
	Mu sync.RWMutex
	Base
	Status      TaskStatus //任务状态
	agent       string     //分发该任务的智能体
	Cluster     int
	Density     float64
	UpTime      int          //任务上线时间
	ChargePoint *ChargePoint //离任务最近充电点
	ChargeDist  float64      //离任务最近充电点距离
	CostPow     float64
	Cars        []Car
	Drones      []DroneCollection
	Workers     []Worker
}
type DroneCollection struct {
	Drone   Drone
	NeedCar bool
}

func NewTaskPoint() *TaskPoint {
	return &TaskPoint{
		Cars:    make([]Car, 0),
		Drones:  make([]DroneCollection, 0),
		Workers: make([]Worker, 0),
	}
}

func (tp *TaskPoint) SetAgent(a string) {
	tp.Mu.Lock()
	defer tp.Mu.Unlock()
	tp.agent = a
}
func (tp *TaskPoint) GetAgent() string {
	return tp.agent
}

func (tp *TaskPoint) AddWorker(a Worker) {
	tp.Workers = append(tp.Workers, a)
}

func (tp *TaskPoint) AddCar(a Car) {
	tp.Cars = append(tp.Cars, a)
}

func (tp *TaskPoint) Init() {
	tp.Cars = make([]Car, 0)
	tp.Drones = make([]DroneCollection, 0)
	tp.Workers = make([]Worker, 0)
}

// func (ms *TaskPoint) getAgents() []Agent {
// 	ms.mu.RLock()
// 	defer ms.mu.RUnlock()
// 	return ms.agents
// }

// func (ms *TaskPoint) SetIsScanIfFalse() bool {
// 	ms.mu.Lock()
// 	defer ms.mu.Unlock()

// 	if !ms.isScan {
// 		ms.isScan = true
// 		return true
// 	}
// 	return false
// }
