package core

type CarEfficiencyResult struct {
	EfficiencyValue float64
	Time            float64 //充电需要多少时间
	StartTime       int     //开始时刻
	NeedTime        float64 //消耗时间
	CarTime         float64 //汽车到达时间
	Car             *Car
	ChargePointInfo *ChargePointInfo
}
type CarEfficiencyResults []CarEfficiencyResult

func (a CarEfficiencyResults) Len() int {
	return len(a)
}
func (a CarEfficiencyResults) Swap(i, j int) {
	a[i], a[j] = a[j], a[i]
}
func (a CarEfficiencyResults) Less(i, j int) bool {
	if a[i].EfficiencyValue != a[j].EfficiencyValue {
		return a[i].EfficiencyValue > a[j].EfficiencyValue
	} else {
		if a[i].Car.Id != a[j].Car.Id {
			return a[i].Car.Id < a[j].Car.Id
		} else {
			return a[i].ChargePointInfo.Id < a[j].ChargePointInfo.Id

		}

	}
}

// GroupAndSortResults 根据汽车分组，并按照效能值从高到低排序
//func GroupAndSortResults(results CarEfficiencyResults) map[*Car]CarEfficiencyResults {
//	groupedResults := make(map[*Car]CarEfficiencyResults)
//
//	// 按照汽车分组
//	for _, result := range results {
//		car := result.Car
//		groupedResults[car] = append(groupedResults[car], result)
//	}
//
//	// 对每组内的效能值进行排序
//	for car, carResults := range groupedResults {
//		sort.Sort(carResults)
//		groupedResults[car] = carResults
//	}
//
//	return groupedResults
//}

// FindHighestEfficiencyDroneWithConflictResolution 找到每辆汽车对应的效能值最高的无人机，并处理无人机冲突，key是无人机uuid
//func FindHighestEfficiencyDroneWithConflictResolution(results CarEfficiencyResults, groupedResults map[*Car]CarEfficiencyResults) (map[string]CarEfficiencyResult, map[string]bool, map[string]bool) {
//	sort.Sort(results)
//
//	highestEfficiencyDrones := make(map[string]CarEfficiencyResult)
//	droneAssigned := make(map[string]bool)
//	carAssigned := make(map[string]bool)
//
//	// 处理无人机冲突
//	for _, result := range results {
//		car := result.Car
//		drone := result.Drone
//		if !droneAssigned[drone.GetUUId()] && !carAssigned[car.GetUUId()] {
//			highestEfficiencyDrones[car.GetUUId()] = result
//			highestEfficiencyDrones[drone.GetUUId()] = result
//			droneAssigned[drone.GetUUId()] = true
//			carAssigned[car.GetUUId()] = true
//		}
//	}
//	return highestEfficiencyDrones, droneAssigned, carAssigned
//}
