package core

import (
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"sort"
	"strconv"
	"strings"
	"time"
)

// 找到离某个点最近的充电点
func FoundNearChargePoisition(x int, y int, chargePoints map[int]*ChargePoint) (int, float64) {
	mindist := math.MaxFloat32
	chargeIndex := -1
	for _, chargePoint := range chargePoints {
		dist := Distance(chargePoint.X, chargePoint.Y, x, y)
		if dist < mindist {
			chargeIndex = chargePoint.Id
			mindist = dist
		}
	}
	return chargeIndex, mindist
}

// distance 计算两个点之间的欧几里得距离
func Distance(x1, y1, x2, y2 int) float64 {
	return math.Sqrt(float64((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)))
}

// CalculateAverageDistance 计算代理位置到所有任务点的平均距离
func CalculateAverageDistance(agentPos Position, taskPoints map[int]*TaskPoint) float64 {
	if len(taskPoints) == 0 {
		return 0.0
	}

	var totalDistance float64
	for _, tp := range taskPoints {
		totalDistance += Distance(agentPos.X, agentPos.Y, tp.Position.X, tp.Position.Y)
	}

	return totalDistance / float64(len(taskPoints))
}

// calculateDensity 使用 KNN 计算每个任务点的密集度
func CalculateDensity(allTaskPoints map[int]*TaskPoint, radius float64) {
	for _, point := range allTaskPoints {
		neighbors := 0
		for _, neighbor := range allTaskPoints {
			if point.Id != neighbor.Id {
				distance := Distance(point.X, point.Y, neighbor.X, neighbor.Y)
				if distance <= radius {
					neighbors++
				}
			}
		}
		// 更新该任务点的密集度（邻居数量作为密集度）
		point.Density = float64(neighbors) / float64(len(allTaskPoints))
	}
}

// 计算潜在任务变化量
func PotentialTaskChange(taskPoints map[int]*TaskPoint, chargePoints map[int]*ChargePoint, drones []*Drone, workers []*Worker, cars []*Car, node GraphNode, actions []Action, index int) float64 {

	avgDrone := 0.0
	avgWorker := 0.0
	avgCar := 0.0

	for _, drone := range drones {
		avgDrone += drone.URget
	}
	avgDrone = avgDrone / float64(len(drones))
	for _, worker := range workers {
		avgWorker += worker.WRget
	}
	avgWorker = avgWorker / float64(len(workers))
	for _, car := range cars {
		avgCar += car.CRget
	}
	//beishu2 := 0.0
	if len(cars) == 0 {
		avgCar = 0
	} else {
		avgCar = avgCar / float64(len(cars))
		//beishu2 = avgDrone / avgCar
	}

	//beishu := avgDrone / avgWorker

	// 计算任务点的覆盖度分数
	calculateCoverageScore := func(tasks map[int]*TaskPoint, drones []*Drone, workers []*Worker, chargeScore map[int]float64) map[int]float64 {

		//记录任务点是否被电量范围覆盖
		m := make(map[int]float64)

		for _, drone := range drones {
			for _, task := range tasks {
				dist := Distance(task.Position.X, task.Position.Y, drone.Position.X, drone.Position.Y)
				if drone.RemainingPower > dist+task.CostPow+task.ChargeDist {
					//m[task.Id]++
					x := drone.RemainingPower / drone.FullPower
					urgent := (1 - math.Exp(-1*x)) / (1 - math.Exp(-1))
					m[task.Id] = urgent + m[task.Id]
				}
			}

		}
		cs := make(map[int]float64)

		for _, task := range tasks {
			score := 0.0
			//minDroneDist := math.MaxFloat64
			//minWorkerDist := math.MaxFloat64
			minWorkerTime := math.MaxFloat64
			minDroneTime := math.MaxFloat64
			for _, drone := range drones {
				dist := Distance(task.Position.X, task.Position.Y, drone.Position.X, drone.Position.Y)
				//if dist < minDroneDist {
				//	minDroneDist = dist
				//}
				time := dist / drone.URget
				if time < minDroneTime {
					minDroneTime = time
				}
			}
			for _, worker := range workers {
				dist := Distance(task.Position.X, task.Position.Y, worker.Position.X, worker.Position.Y)
				time := dist / worker.WRget
				//if dist < minWorkerDist {
				//	minWorkerDist = dist
				//}
				if time < minWorkerTime {
					minWorkerTime = time
				}
			}

			//score = m[task.Id] / (minDroneDist + beishu*minWorkerDist + alpha)
			score = math.Pow(m[task.Id], 0.6) / (math.Pow(m[task.Id], 0.6) + (minDroneTime+minWorkerTime)/10)

			cs[task.Id] = score
		}
		return cs
	}

	// 计算充电点的覆盖度分数
	calculateChargeCoverageScore := func(charges map[int]*ChargePoint, drones []*Drone, cars []*Car) map[int]float64 {

		//记录充电点是否被低电量覆盖
		m := make(map[int]float64)

		for _, drone := range drones {

			position := drone.Position
			remainingPower := drone.RemainingPower

			for _, charge := range charges {
				dist := Distance(charge.Position.X, charge.Position.Y, position.X, position.Y)
				if remainingPower > dist {
					x := drone.RemainingPower / drone.FullPower
					urgent := (1 - math.Exp(-1*x)) / (1 - math.Exp(-1))
					m[charge.Id] = urgent + m[charge.Id]
				}
			}
		}

		for _, action := range actions { //获取以后的位置和电量信息
			position := Position{}
			remainingPower := 0.0
			if action.Drone != nil {
				position = action.Destination
				remainingPower = action.Drone.RemainingPower - action.CostPow
			}
			if float64(action.StartTime)+action.Time > float64(index*DecideTime)+node.Time { //car只负责到达能管到的
				continue
			}
			for _, charge := range charges {
				dist := Distance(charge.Position.X, charge.Position.Y, position.X, position.Y)
				if remainingPower > dist {
					x := remainingPower / action.Drone.FullPower
					urgent := (1 - math.Exp(-1*x)) / (1 - math.Exp(-1))
					m[charge.Id] = urgent + m[charge.Id]
				}
			}
		}
		cs := make(map[int]float64)

		for _, charge := range charges {
			score := 0.0
			//minDroneDist := math.MaxFloat64
			//minCarDist := math.MaxFloat64
			minCarTime := math.MaxFloat64
			//urgent := 0.0
			//for _, drone := range drones {
			//	dist := Distance(charge.Position.X, charge.Position.Y, drone.Position.X, drone.Position.Y)
			//	if dist < minDroneDist {
			//		minDroneDist = dist
			//	}
			//}
			for _, car := range cars {
				dist := Distance(charge.Position.X, charge.Position.Y, car.Position.X, car.Position.Y)
				time := dist / car.CRget
				//if dist < minWorkerDist {
				//	minWorkerDist = dist
				//}
				if time < minCarTime {
					minCarTime = time
				}
			}
			//score = m[charge.Id] / (minCarDist + alpha)
			score = math.Pow(m[charge.Id], 0.6) / (math.Pow(m[charge.Id], 0.6) + (minCarTime)/10)
			cs[charge.Id] = score
		}
		return cs
	}

	//theload := 1 / (beishu*float64(DecideTime)*avgWorker + float64(DecideTime)*avgDrone + alpha)
	//theload2 := 1 / (float64(DecideTime)*avgCar + alpha)
	// 执行行动前的覆盖度分数和潜在任务数量
	initialChargeCS := calculateChargeCoverageScore(chargePoints, drones, cars)

	initialCS := calculateCoverageScore(taskPoints, drones, workers, initialChargeCS)

	//initialPotentialTasks := countPotentialTasks(initialCS, theload)
	//initialPotentialCharges := countPotentialTasks(initialChargeCS, theload2)

	// 创建智能体和任务的副本，模拟执行行动后的状态
	clonedDrones := cloneDrones(drones)
	clonedWorkers := cloneWorkers(workers)
	clonedTasks := cloneTaskPoints(taskPoints)
	clonedCars := cloneCars(cars)

	// 在副本上执行行动
	updateStateWithAction(clonedDrones, clonedWorkers, clonedCars, node)
	// 执行<car,c><u,car,c>后的充电点的覆盖度分数
	finalChargeCS := calculateChargeCoverageScore(chargePoints, clonedDrones, clonedCars)
	// 执行行动后的覆盖度分数和潜在任务数量
	finalCS := calculateCoverageScore(clonedTasks, clonedDrones, clonedWorkers, finalChargeCS)

	//finalPotentialTasks := countPotentialTasks(finalCS, theload)
	//finalPotentialCharges := countPotentialTasks(finalChargeCS, theload2)

	count := 0.0

	if node.NodeType == DroneToTask || node.NodeType == DroneWorkerToTask || node.NodeType == WorkerToTask || node.NodeType == WorkerToCharge {
		for id := range finalCS {
			if finalCS[id]-initialCS[id] > 0 {
				count = count + finalCS[id]
			}
		}
		//return float64(finalPotentialTasks - initialPotentialTasks)
	} else {
		for id := range finalChargeCS {
			if finalChargeCS[id]-initialChargeCS[id] > 0 {
				count = count + finalChargeCS[id]
			}
		}
		//return float64(finalPotentialCharges - initialPotentialCharges)
	}

	//if node.NodeType == CarToChargePoint {
	//	return finalChargeCS[node.ChargePoint.Id]
	//}
	// 返回潜在任务变化量
	return count
}

// 统计潜在任务数量（覆盖度分数超过阈值的任务点）
func countPotentialTasks(cs map[int]float64, theload float64) int {
	threshold := 0.0 // 设定覆盖度分数阈值
	count := 0

	var csValue []float64
	for _, values := range cs {
		csValue = append(csValue, values)
	}
	//medianCs := calculateMedian(csValue)
	//
	//stdDev := calculateStdDev(csValue)
	threshold = theload

	for _, score := range cs {
		if score > threshold && score != 0 {
			count++
		}
	}

	return count
}

// 在副本上模拟更新状态
func updateStateWithAction(drones []*Drone, workers []*Worker, cars []*Car, node GraphNode) {
	switch node.NodeType {
	case DroneToTask:
		for _, drone := range drones {
			if drone.Id == node.Drone.Id {
				drone.Position = node.TaskPoint.Position
				drone.RemainingPower -= node.CostPower
			}
		}
	case WorkerToTask:
		for _, worker := range workers {
			if worker.Id == node.Worker.Id {
				worker.Position = node.TaskPoint.Position
			}
		}
	case DroneWorkerToTask:
		for _, drone := range drones {
			if drone.Id == node.Drone.Id {
				drone.Position = node.TaskPoint.Position
				drone.RemainingPower -= node.CostPower
			}
		}
		for _, worker := range workers {
			if worker.Id == node.Worker.Id {
				worker.Position = node.TaskPoint.Position
			}
		}
	case DroneCarToChargePoint:
		for _, drone := range drones {
			if drone.Id == node.Drone.Id {
				drone.Position = node.ChargePoint.Position
				drone.RemainingPower = drone.FullPower
			}
		}
	case WorkerToCharge:
		for _, worker := range workers {
			if worker.Id == node.Worker.Id {
				worker.Position = node.ChargePoint.Position
			}
		}
	case CarToChargePoint:
		for _, car := range cars {
			if car.Id == node.Car.Id {
				car.Position = node.ChargePoint.Position
			}
		}
	case DroneToChargePoint:
		for _, drone := range drones {
			if drone.Id == node.Drone.Id {
				drone.Position = node.ChargePoint.Position
				drone.RemainingPower -= node.CostPower
			}
		}
	default:
		// 其他行动类型的处理逻辑可以扩展
	}
}

// 中位数
func calculateMedian(cs []float64) float64 {
	sort.Float64s(cs)
	n := len(cs)
	if n%2 == 1 {
		return cs[n/2]
	}
	return (cs[n/2-1] + cs[n/2]) / 2.0
}

// 方差
func calculateStdDev(cs []float64) float64 {
	sort.Float64s(cs)
	mean := 0.0
	for _, value := range cs {
		mean += value
	}
	mean = mean / float64(len(cs))

	sum := 0.0
	for _, value := range cs {
		sum += (value - mean) * (value - mean)
	}
	variance := sum / float64(len(cs))
	return math.Sqrt(variance)
}

// 克隆任务点的副本
func cloneTaskPoints(tasks map[int]*TaskPoint) map[int]*TaskPoint {
	clonedTasks := make(map[int]*TaskPoint)
	for id, task := range tasks {
		clonedTask := *task
		clonedTasks[id] = &clonedTask
	}
	return clonedTasks
}

// 克隆无人机的副本
func cloneDrones(drones []*Drone) []*Drone {
	clonedDrones := make([]*Drone, len(drones))
	for i, drone := range drones {
		clonedDrone := *drone
		clonedDrones[i] = &clonedDrone
	}
	return clonedDrones
}

// 克隆无人机的副本
func cloneCars(cars []*Car) []*Car {
	clonedCars := make([]*Car, len(cars))
	for i, car := range cars {
		clonedCar := *car
		clonedCars[i] = &clonedCar
	}
	return clonedCars
}

// 克隆工作人员的副本
func cloneWorkers(workers []*Worker) []*Worker {
	clonedWorkers := make([]*Worker, len(workers))
	for i, worker := range workers {
		clonedWorker := *worker
		clonedWorkers[i] = &clonedWorker
	}
	return clonedWorkers
}

func parseDurationFromMinutes(minutes int) time.Duration {
	return time.Duration(minutes) * time.Minute
}

func DataProcess(name string) []Folder {
	// 打开CSV文件
	file, err := os.Open(name)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	reader := csv.NewReader(file)

	// 读取CSV文件
	records, err := reader.ReadAll()
	if err != nil {
		panic(err)
	}

	filesName := make([]Folder, 0)

	// 遍历CSV文件中的每一行，跳过标题行
	for _, record := range records[1:] { // 假设第一行是标题行
		// 解析文件夹名称
		folderName := record[0]

		// 解析DecideTime（分钟）
		decideTimeStr := record[1]
		DecideTime, err := extractNumberFromTime(decideTimeStr)
		if err != nil {
			fmt.Printf("Error parsing DecideTime: %v\n", err)
			continue
		}

		// 解析TimeLimit（小时）
		timeLimitStr := record[2]
		timeLimitHours, err := strconv.Atoi(strings.TrimSuffix(timeLimitStr, "h"))
		if err != nil {
			fmt.Printf("Error parsing TimeLimit: %v\n", err)
			continue
		}

		// 将DecideTime和TimeLimit转换为time.Duration
		decideTimeDuration := parseDurationFromMinutes(DecideTime)
		timeLimitDuration := time.Duration(timeLimitHours) * time.Hour

		// 计算DecideCount
		DecideCount := int(timeLimitDuration / decideTimeDuration)

		folder := Folder{Name: folderName, DecideTime: DecideTime, DecideCount: DecideCount}
		filesName = append(filesName, folder)
	}

	return filesName
}

// updateResultInThirdColumn 根据文件夹名称更新第三列的结果
// updateResultInThirdColumn 根据文件夹名称更新第三列的结果
func UpdateResultInThirdColumn(records [][]string, name string, folder Folder, result, maxFl, costTime float64) [][]string {

	// 遍历工作表中的每一行，找到对应的文件夹名称并更新第三列
	for i, row := range records {
		if i == 0 {
			continue
		}
		// 解析文件夹名称
		folderName := row[0]
		// 解析DecideTime（分钟）
		decideTimeStr := row[1]
		decideTime, err := extractNumberFromTime(decideTimeStr)
		if err != nil {
			fmt.Printf("Error parsing DecideTime: %v\n", err)
			continue
		}

		// 解析TimeLimit（小时）
		timeLimitStr := row[2]
		timeLimitHours, err := strconv.Atoi(strings.TrimSuffix(timeLimitStr, "h"))
		if err != nil {
			fmt.Printf("Error parsing TimeLimit: %v\n", err)
			continue
		}

		// 将DecideTime和TimeLimit转换为time.Duration
		decideTimeDuration := parseDurationFromMinutes(DecideTime)
		timeLimitDuration := time.Duration(timeLimitHours) * time.Hour

		// 计算DecideCount
		decideCount := int(timeLimitDuration / decideTimeDuration)
		if folderName == folder.Name && decideTime == folder.DecideTime && decideCount == folder.DecideCount {
			// 更新第三列的值
			records[i][3] = fmt.Sprintf("%.4f", result)
			records[i][4] = fmt.Sprintf("%.4f", maxFl)
			records[i][5] = fmt.Sprintf("%.4f", costTime)
		}
	}
	// 循环结束后保存文件，这样只更新特定单元格，其他内容不变

	return records
}

// extractNumberFromTime 从时间字符串中提取数字
func extractNumberFromTime(timeStr string) (int, error) {
	// 去除字符串中的非数字字符，只保留数字
	numberStr := strings.ReplaceAll(timeStr, "min", "")
	numberStr = strings.ReplaceAll(numberStr, "h", "")
	return strconv.Atoi(numberStr)
}
func QuChong(records [][]string, folder Folder) bool {
	// 遍历工作表中的每一行，找到对应的文件夹名称并更新第三列
	for i, row := range records {
		if i == 0 {
			continue
		}
		// 解析文件夹名称
		folderName := row[0]
		// 解析DecideTime（分钟）
		decideTimeStr := row[1]
		decideTime, err := extractNumberFromTime(decideTimeStr)
		if err != nil {
			fmt.Printf("Error parsing DecideTime: %v\n", err)
			continue
		}

		// 解析TimeLimit（小时）
		timeLimitStr := row[2]
		timeLimitHours, err := strconv.Atoi(strings.TrimSuffix(timeLimitStr, "h"))
		if err != nil {
			fmt.Printf("Error parsing TimeLimit: %v\n", err)
			continue
		}

		// 将DecideTime和TimeLimit转换为time.Duration
		decideTimeDuration := parseDurationFromMinutes(DecideTime)
		timeLimitDuration := time.Duration(timeLimitHours) * time.Hour

		// 计算DecideCount
		decideCount := int(timeLimitDuration / decideTimeDuration)
		if folderName == folder.Name && decideTime == folder.DecideTime && decideCount == folder.DecideCount {
			if len(row[4]) > 0 {
				return true
			}
		}
	}
	// 循环结束后保存文件，这样只更新特定单元格，其他内容不变

	return false
}
