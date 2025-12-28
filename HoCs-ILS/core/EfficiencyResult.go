package core

// 存储联盟效能函数结果的数据结构
type EfficiencyResult struct {
	TaskID          int
	EfficiencyValue float64
	NeedCar         bool
	NeedCharge      bool    //代表执行完任务后需要充电
	Time            float64 //完成任务需要多久时间
	StartTime       int     //开始时刻
	Distance        float64
	Car             *Car
	Drone           *Drone
	Worker          *Worker
}

type ByAllEfficiencyResult []EfficiencyResult

func (a ByAllEfficiencyResult) Len() int {
	return len(a)
}
func (a ByAllEfficiencyResult) Swap(i, j int) {
	a[i], a[j] = a[j], a[i]
}
func (a ByAllEfficiencyResult) Less(i, j int) bool {
	if a[i].EfficiencyValue != a[j].EfficiencyValue {
		return a[i].EfficiencyValue > a[j].EfficiencyValue
	} else {
		//	if a[i].TaskID != a[j].TaskID {
		//		return a[i].TaskID < a[j].TaskID
		//	} else {
		//		if a[i].Worker.Id != a[j].Worker.Id {
		//			return a[i].Worker.Id < a[j].Worker.Id
		//		} else {
		//			return a[i].Drone.Id < a[j].Drone.Id
		//		}
		//	}
		return a[i].EfficiencyValue > a[j].EfficiencyValue
	}
}
