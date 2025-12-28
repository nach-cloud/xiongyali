package core

import "sync"

type EfficiencyResults struct {
	mu               sync.Mutex
	efficiencyResult []EfficiencyResult
}

func (eff *EfficiencyResults) Append(result EfficiencyResult) {
	eff.mu.Lock()
	defer eff.mu.Unlock()
	eff.efficiencyResult = append(eff.efficiencyResult, result)
}

func (eff *EfficiencyResults) Get() []EfficiencyResult {
	eff.mu.Lock()
	defer eff.mu.Unlock()
	return eff.efficiencyResult
}

func (eff *EfficiencyResults) Init() {
	eff.efficiencyResult = make([]EfficiencyResult, 0)
}
func (eff *EfficiencyResults) Delete(taskid int) {
	for _, e := range eff.Get() {
		if e.TaskID == taskid {
			eff.efficiencyResult = removeElement(eff.efficiencyResult, e, func(result EfficiencyResult, result2 EfficiencyResult) bool {
				return result.Drone.GetUUId() == result2.Drone.GetUUId() && result.Worker.GetUUId() == result2.Worker.GetUUId()
			})
		}
	}
}

// 删除指定值
func removeElement[T any](arr []T, element T, equalFunc func(T, T) bool) []T {
	j := 0
	for _, v := range arr {
		if !equalFunc(element, v) {
			arr[j] = v
			j++
		}
	}
	return arr[:j]
}
