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
func softplus(x float64) float64 {
	return math.Log(1 + math.Exp(x))
}

// 计算潜在任务变化量
// 计算潜在任务变化量
func PotentialTaskChange(taskPoints map[int]*TaskPoint, chargePoints map[int]*ChargePoint, node GraphNode, beforeDutL map[int]float64, beforeDwtL map[int]float64, beforeDuchL map[int]float64, beforeDvchL map[int]float64) float64 {

	// 简化版softmax计算函数
	softmax := func(values []float64) []float64 {
		result := make([]float64, len(values))
		sum := 0.0
		max := math.Inf(-1)

		for _, v := range values {
			if v > max {
				max = v
			}
		}

		for i, v := range values {
			result[i] = math.Exp(v - max)
			sum += result[i]
		}

		if sum > 0 {
			for i := range result {
				result[i] /= sum
			}
		}

		return result
	}

	//sigmoid := func(x float64) float64 {
	//	return 1.0 / (1.0 + math.Exp(-x))
	//}
	// 在副本上执行行动
	var clonedDrone Drone
	var clonedWorker Worker
	var clonedCar Car
	// 根据节点类型更新状态
	if node.Drone != nil {
		clonedDrone = *node.Drone
	}
	if node.Worker != nil {
		clonedWorker = *node.Worker
	}
	if node.Car != nil {
		clonedCar = *node.Car
	}

	updateStateWithAction(&clonedDrone, &clonedWorker, &clonedCar, node)
	// 根据公式计算各类收益
	var result float64 = 0

	// 工人收益计算
	if node.Worker != nil {
		values := make([]float64, 0)
		values2 := make([]float64, 0)
		taskIDs := make([]int, 0)

		for id, task := range taskPoints {
			dist := Distance(task.Position.X, task.Position.Y, node.Worker.Position.X, node.Worker.Position.Y)
			dist2 := Distance(task.Position.X, task.Position.Y, clonedWorker.Position.X, clonedWorker.Position.Y)
			values = append(values, -computeDecideTime(dist/node.Worker.WRget))
			values2 = append(values2, -computeDecideTime(dist2/node.Worker.WRget))
			taskIDs = append(taskIDs, id)
		}

		probs := softmax(values)
		probs2 := softmax(values2)

		beforeSum := 0.0
		afterSum := 0.0
		for i, id := range taskIDs {
			beforeSum += probs[i] * beforeDutL[id]
			afterSum += probs2[i] * beforeDutL[id]
		}

		if beforeSum > 0 {

			icmW := softplus((beforeSum - afterSum) / beforeSum)
			result += icmW
		}
	}

	// 无人机收益计算
	if node.Drone != nil {
		// 任务收益
		tValues := make([]float64, 0)
		tValues2 := make([]float64, 0)
		tIDs := make([]int, 0)
		tIDs2 := make([]int, 0)

		for id, task := range taskPoints {
			dist := Distance(task.Position.X, task.Position.Y, node.Drone.Position.X, node.Drone.Position.Y)
			if node.Drone.RemainingPower > dist+task.CostPow+task.ChargeDist {
				tValues = append(tValues, -computeDecideTime(dist/node.Drone.URget))
				tIDs = append(tIDs, id)
			}
			dist2 := Distance(task.Position.X, task.Position.Y, clonedDrone.Position.X, clonedDrone.Position.Y)
			if clonedDrone.RemainingPower > dist2+task.CostPow+task.ChargeDist {
				tValues2 = append(tValues2, -computeDecideTime(dist2/node.Drone.URget))
				tIDs2 = append(tIDs2, id)
			}

		}
		var icmUT float64 = 0
		if len(tValues) > 0 {
			tProbs := softmax(tValues)
			tProbs2 := softmax(tValues2)

			beforeTSum := 0.0
			afterTSum := 0.0
			for i, id := range tIDs {
				beforeTSum += tProbs[i] * beforeDwtL[id]
			}
			for i, id := range tIDs2 {
				afterTSum += tProbs2[i] * beforeDwtL[id]
			}
			if beforeTSum > 0 {

				icmUT = softplus(beforeTSum-afterTSum) / (beforeTSum)

			}

		}
		// 充电收益
		cValues := make([]float64, 0)
		cValues2 := make([]float64, 0)
		cIDs := make([]int, 0)
		cIDs2 := make([]int, 0)

		for id, charge := range chargePoints {
			dist := Distance(charge.Position.X, charge.Position.Y, node.Drone.Position.X, node.Drone.Position.Y)
			if node.Drone.RemainingPower > dist {
				cValues = append(cValues, -computeDecideTime(dist/node.Drone.URget))
				cIDs = append(cIDs, id)
			}
			dist2 := Distance(charge.Position.X, charge.Position.Y, clonedDrone.Position.X, clonedDrone.Position.Y)
			if clonedDrone.RemainingPower > dist2 {
				cValues2 = append(cValues2, -computeDecideTime(dist2/node.Drone.URget))
				cIDs2 = append(cIDs2, id)
			}
		}

		var icmUC float64 = 0
		if len(cValues) > 0 {
			cProbs := softmax(cValues)
			cProbs2 := softmax(cValues2)

			beforeCSum := 0.0
			afterCSum := 0.0
			for i, id := range cIDs {
				beforeCSum += cProbs[i] * beforeDvchL[id]
			}
			for i, id := range cIDs2 {
				afterCSum += cProbs2[i] * beforeDvchL[id]
			}
			if beforeCSum > 0 {
				icmUC = softplus(beforeCSum-afterCSum) / beforeCSum
			}
		}

		// 无人机综合收益
		powerRatio := clonedDrone.RemainingPower / clonedDrone.FullPower
		icmU := (powerRatio*icmUT + (1-powerRatio)*icmUC)
		//icmU := (icmUT + icmUC) / 2
		//这里前面加power会跌百分之十
		result += icmU
	}

	// 汽车收益计算
	if node.Car != nil {
		values := make([]float64, 0)
		values2 := make([]float64, 0)
		chargeIDs := make([]int, 0)

		for id, charge := range chargePoints {
			dist := Distance(charge.Position.X, charge.Position.Y, node.Car.Position.X, node.Car.Position.Y)
			dist2 := Distance(charge.Position.X, charge.Position.Y, clonedCar.Position.X, clonedCar.Position.Y)
			values = append(values, -computeDecideTime(dist/node.Car.CRget))
			values2 = append(values2, -computeDecideTime(dist2/node.Car.CRget))
			chargeIDs = append(chargeIDs, id)
		}

		if len(values) > 0 {
			probs := softmax(values)
			probs2 := softmax(values2)

			beforeSum := 0.0
			afterSum := 0.0
			for i, id := range chargeIDs {
				beforeSum += probs[i] * beforeDuchL[id]
				afterSum += probs2[i] * beforeDuchL[id]
			}

			if beforeSum > 0 {
				icmV := softplus((beforeSum - afterSum) / beforeSum)
				result += icmV
			}
		}
	}
	// 根据节点类型添加额外收益
	switch node.NodeType {
	case DroneWorkerToTask:
		// 三维期望匹配额外收益
		if node.Drone != nil && node.Worker != nil {
			result += softplus(100) // 协同收益
		}
	case DroneCarToChargePoint:
		// 充电三维期望收益
		if node.Drone != nil && node.Car != nil {
			powerRatio := node.Drone.RemainingPower / node.Drone.FullPower
			urgencyFactor := softplus(10 + (math.Exp((1 - powerRatio))))
			result += urgencyFactor
		}
	}
	return result
}

func computeDecideTime(coaTaskTime float64) float64 {
	decidecount := int(coaTaskTime) / DecideTime //33 33/10=3
	taskCompleteDecideTime := (decidecount + 1) * DecideTime
	return float64(taskCompleteDecideTime)
}

// 在副本上模拟更新状态
// 在副本上模拟更新状态
func updateStateWithAction(drone *Drone, worker *Worker, car *Car, node GraphNode) {
	switch node.NodeType {
	case DroneToTask:
		drone.Position = node.TaskPoint.Position
		drone.RemainingPower -= node.CostPower
	case WorkerToTask:
		worker.Position = node.TaskPoint.Position
	case DroneWorkerToTask:
		drone.Position = node.TaskPoint.Position
		drone.RemainingPower -= node.CostPower
		worker.Position = node.TaskPoint.Position
	case DroneCarToChargePoint:
		drone.Position = node.ChargePoint.Position
		drone.RemainingPower -= node.CostPower
		car.Position = node.ChargePoint.Position
	case WorkerToCharge:
		worker.Position = node.ChargePoint.Position
	case CarToChargePoint:
		car.Position = node.ChargePoint.Position
	case DroneToChargePoint:
		drone.Position = node.ChargePoint.Position
		drone.RemainingPower -= node.CostPower
	default:
		// 其他行动类型的处理逻辑可以扩展
	}
}
func CalculateCoverageScore(drones []*Drone, workers []*Worker, cars []*Car, taskPoints map[int]*TaskPoint, chargePoints map[int]*ChargePoint) (map[int]float64, map[int]float64, map[int]float64, map[int]float64) {
	// 初始化各项指标
	DutL := make(map[int]float64)  // 无人机任务延迟
	DwtL := make(map[int]float64)  // 工人任务延迟
	DuchL := make(map[int]float64) // 无人机充电延迟
	DvchL := make(map[int]float64) // 车辆充电延迟

	// 1. 计算任务相关指标
	for _, task := range taskPoints {
		// 无人机任务指标
		count := 0.0
		distSum := 0.0
		for _, drone := range drones {
			dist := Distance(task.Position.X, task.Position.Y, drone.Position.X, drone.Position.Y)
			// 约束条件(6)：无人机能否到达该任务点
			if drone.RemainingPower > dist+task.CostPow+task.ChargeDist {
				powerRatio := drone.RemainingPower / drone.FullPower
				energyTerm := (1 - math.Exp(-powerRatio)) / (1 - math.Exp(-1))
				distSum += (computeDecideTime(dist / drone.URget)) / energyTerm
				count++
			}
		}
		if count > 0 {
			DutL[task.Id] = distSum / count
		} else {
			DutL[task.Id] = 0
		}

		// 工人任务指标
		wdistSum := 0.0
		count = 0.0
		for _, worker := range workers {
			dist := Distance(task.Position.X, task.Position.Y, worker.Position.X, worker.Position.Y)
			wdistSum += computeDecideTime(dist / worker.WRget)
			count++
		}
		if count > 0 {
			DwtL[task.Id] = wdistSum / count
		} else {
			DwtL[task.Id] = 0
		}

	}

	// 2. 计算充电相关指标
	for _, charge := range chargePoints {
		// 无人机充电指标
		count := 0.0
		distSum := 0.0
		for _, drone := range drones {
			dist := Distance(charge.Position.X, charge.Position.Y, drone.Position.X, drone.Position.Y)
			// 约束条件(5)：无人机能否到达该充电点
			if drone.RemainingPower > dist {
				powerRatio := drone.RemainingPower / drone.FullPower
				energyTerm := (1 - math.Exp(-(1 - powerRatio))) / (1 - math.Exp(-1))
				if energyTerm != 0 {
					count++
					distSum += (computeDecideTime(dist / drone.URget)) / energyTerm
				}
			}
		}
		if count > 0 {
			DuchL[charge.Id] = distSum / count
		} else {
			DuchL[charge.Id] = 0
		}

		// 车辆充电指标
		cdistSum := 0.0
		count = 0.0
		for _, car := range cars {
			dist := Distance(charge.Position.X, charge.Position.Y, car.Position.X, car.Position.Y)
			cdistSum += computeDecideTime(dist / car.CRget)
			count++
		}
		if count > 0 {
			DvchL[charge.Id] = cdistSum
		} else {
			DvchL[charge.Id] = 0
		}

	}

	return DutL, DwtL, DuchL, DvchL
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
