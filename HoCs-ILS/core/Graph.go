package core

import (
	"fmt"
	"math"
	"time"
)

// Graph 结构体表示有向无权图
type Graph struct {
	Nodes           map[string]*GraphNode      // 节点集合（组合 -> 权重）
	Edges           map[string]map[string]bool // 边集合（节点 -> 其他相连节点的映射）
	BestKnownWeight float64
}
type GraphNode struct {
	NodeKey     string
	NodeType    NodeType     // 节点类型，表示这个节点的情况
	Drone       *Drone       // 无人机，可能为空
	Car         *Car         // 汽车，可能为空
	Worker      *Worker      // 工作人员，可能为空
	TaskPoint   *TaskPoint   // 任务点，可能为空
	ChargePoint *ChargePoint // 充电点，可能为空
	Weight      float64      // 节点权值
	CostPower   float64      //消耗电量
	Time        float64
}

// NewGraph 初始化图
func NewGraph() *Graph {
	return &Graph{
		Nodes: make(map[string]*GraphNode),
		Edges: make(map[string]map[string]bool),
	}
}

// 添加节点
func (g *Graph) AddNode(node GraphNode) {
	g.Nodes[node.NodeKey] = &node
}

// 添加边
func (g *Graph) AddEdge(from, to string) {
	if g.Edges[from] == nil {
		g.Edges[from] = make(map[string]bool)
	}
	if g.Edges[to] == nil {
		g.Edges[to] = make(map[string]bool)
	}
	g.Edges[from][to] = true
	g.Edges[to][from] = true
}

// 判断是否有边
func (g *Graph) HasEdge(from, to string) bool {
	return g.Edges[from][to]
}

func isConflict(node1, node2 *GraphNode) bool {
	// 检查两个节点是否涉及相同的充电点（如果相关）
	if node1.ChargePoint != nil && node2.ChargePoint != nil && node1.ChargePoint.Id == node2.ChargePoint.Id {
		if node1.Car != nil && node2.Car != nil {
			if node1.Car.UUID == node2.Car.UUID && node1.NodeType == DroneCarToChargePoint && node2.NodeType == DroneCarToChargePoint {
				return false
			} else {
				return true
			}
		} else {
			return false
		}
	}

	// 检查两个节点是否涉及相同的任务点
	if node1.TaskPoint != nil && node2.TaskPoint != nil && node1.TaskPoint.Id == node2.TaskPoint.Id {
		if node1.NodeType == DroneToTask && node2.NodeType == DroneWorkerToTask || node1.NodeType == DroneWorkerToTask && node2.NodeType == DroneToTask {
			if node1.Drone.Id != node2.Drone.Id { //<u1,w1,t1>与<U2,t1>不冲突
				return false
			}
		}
		if node1.NodeType == WorkerToTask && node2.NodeType == DroneWorkerToTask || node1.NodeType == DroneWorkerToTask && node2.NodeType == WorkerToTask {
			if node1.Worker.Id != node2.Worker.Id {
				return false
			}
		}
		return true
	}

	// 检查两个节点是否包含相同的无人机
	if node1.Drone != nil && node2.Drone != nil && node1.Drone.UUID == node2.Drone.UUID {
		return true
	}

	// 检查两个节点是否包含相同的汽车
	if node1.Car != nil && node2.Car != nil && node1.Car.UUID == node2.Car.UUID {
		return true
	}

	// 检查两个节点是否包含相同的工作人员
	if node1.Worker != nil && node2.Worker != nil && node1.Worker.UUID == node2.Worker.UUID {
		return true
	}

	// 如果以上条件都不满足，则认为两个节点不冲突
	return false
}

func BuildGraph(taskPoints map[int]*TaskPoint, drones []*Drone, cars []*Car, workers []*Worker, chargePoints map[int]*ChargePoint, actions []Action, index int) *Graph {
	graph := NewGraph()
	hashmap := make(map[int]bool)
	//taskPointUavCount := make(map[int]int) //有多少个无人机可以到达任务点
	//uavCanProcessTask := make(map[int]int) //无人机可以到达多少个任务点
	//chargePointCount := make(map[int]int)  //充电点可能有多少个无人机
	workerMinSpeed := math.MaxFloat64
	beforeDutL, beforeDwtL, beforeDuchL, beforeDvchL := CalculateCoverageScore(drones, workers, cars, taskPoints, chargePoints)

	for _, worker := range workers {
		if worker.GetSpeed() < workerMinSpeed {
			workerMinSpeed = worker.GetSpeed()
		}
	}
	if workerMinSpeed == math.MaxFloat64 {
		workerMinSpeed = 0.01
	}
	// 1. 智能体原地不动
	for _, drone := range drones {
		node := GraphNode{
			NodeType: DroneIdle,
			Drone:    drone,
			Weight:   softplus(0),
			NodeKey:  fmt.Sprintf("<u%d>", drone.Id),
		}
		graph.AddNode(node)
		//2. 无人机单独执行任务的情况
		for _, taskPoint := range taskPoints {
			droneToTaskDist := Distance(drone.X, drone.Y, taskPoint.X, taskPoint.Y)
			droneToTaskDistTime := (droneToTaskDist / drone.URget)
			decidecount := math.Ceil(droneToTaskDistTime / float64(DecideTime)) //33 33/10=3
			taskCompleteDecideTime := (decidecount) * float64(DecideTime)
			if drone.RemainingPower < droneToTaskDist+taskPoint.ChargeDist { //保证等到完成任务后的下一个决策时刻是可以续航的
				continue
			}
			node = GraphNode{
				NodeType:  DroneToTask,
				Drone:     drone,
				TaskPoint: taskPoint,
				//Weight:    CalculateEfficiencyForDroneToTask(float64(taskCompleteDecideTime)),
				Weight:    0,
				NodeKey:   fmt.Sprintf("<u%d,t%d>", drone.Id, taskPoint.Id),
				Time:      float64(taskCompleteDecideTime),
				CostPower: droneToTaskDist,
			}
			changePosition := PotentialTaskChange(taskPoints, chargePoints, node, beforeDutL, beforeDwtL, beforeDuchL, beforeDvchL)
			node.Weight = changePosition / math.Max(decidecount, 1)
			//if node.Weight != 0 {
			graph.AddNode(node)
			//Dronechange := CalculateAvgDistanceChange(drone, taskPoint, taskPoints)
			//3.无人机和工作人员一起执行任务的节点
			for _, worker := range workers {
				workerToTaskDist := Distance(worker.X, worker.Y, taskPoint.X, taskPoint.Y)
				workerToTaskDistTime := (workerToTaskDist / worker.WRget)
				taskPointNeedTime := taskPoint.CostPow / drone.URget
				coaTaskTime := math.Max(workerToTaskDistTime, droneToTaskDistTime) + taskPointNeedTime
				decidecount := math.Ceil((coaTaskTime / float64(DecideTime))) //33 33/10=3
				taskCompleteDecideTime = (decidecount) * float64(DecideTime)
				if drone.RemainingPower < droneToTaskDist+taskPoint.ChargeDist+taskPoint.CostPow { //保证等到完成任务后的下一个决策时刻是可以续航的
					continue
				}

				//WorkerChange := CalculateAvgDistanceChange(worker, taskPoint, taskPoints)

				node = GraphNode{
					NodeType:  DroneWorkerToTask,
					Drone:     drone,
					Worker:    worker,
					TaskPoint: taskPoint,
					//Weight:    CalculateEfficiencyForDroneWorkerToTask(float64(taskCompleteDecideTime)),
					Weight:    0,
					NodeKey:   fmt.Sprintf("<u%d,w%d,t%d>", drone.Id, worker.Id, taskPoint.Id),
					Time:      float64(taskCompleteDecideTime),
					CostPower: droneToTaskDist + taskPoint.CostPow,
				}
				changePosition = PotentialTaskChange(taskPoints, chargePoints, node, beforeDutL, beforeDwtL, beforeDuchL, beforeDvchL)
				node.Weight = changePosition / math.Max(decidecount, 1)
				graph.AddNode(node)
			}
		}

		for _, chargePoint := range chargePoints {
			droneToChargePoint := Distance(chargePoint.X, chargePoint.Y, drone.X, drone.Y)
			costTime := droneToChargePoint / drone.URget
			decidecount := math.Ceil((costTime) / float64(DecideTime)) //33 33/10=3
			if drone.RemainingPower < droneToChargePoint {             //无人机无法到达该充电点
				continue
			}
			// 5. 无人机和汽车一起前往充电点的节点
			for _, car := range cars {
				carToChargePoint := Distance(chargePoint.X, chargePoint.Y, car.X, car.Y) / car.CRget
				coaTaskTime := math.Max(carToChargePoint, costTime) + drone.FullPower/car.ChargePow
				decidecount := math.Ceil((coaTaskTime) / float64(DecideTime)) //33 33/10=3

				node = GraphNode{
					NodeType:    DroneCarToChargePoint,
					Drone:       drone,
					Car:         car,
					ChargePoint: chargePoint,
					Time:        decidecount * float64(DecideTime),
					Weight:      0,
					NodeKey:     fmt.Sprintf("<u%d,car%d,c%d>", drone.Id, car.Id, chargePoint.Id),
				}
				changePosition := PotentialTaskChange(taskPoints, chargePoints, node, beforeDutL, beforeDwtL, beforeDuchL, beforeDvchL)
				node.Weight = changePosition / math.Max(decidecount, 1)
				graph.AddNode(node)
				//graph.AddNode(node)
				hashmap[chargePoint.Id] = true
			}
			// 4. 无人机单独前往充电点的节点
			if !hashmap[chargePoint.Id] { //这个充电点没有被任何<u1,c1,car1>分配
				node = GraphNode{
					NodeType:    DroneToChargePoint,
					Drone:       drone,
					ChargePoint: chargePoint,
					Time:        (decidecount * float64(DecideTime)),
					Weight:      0,
					NodeKey:     fmt.Sprintf("<u%d,c%d>", drone.Id, chargePoint.Id),
					CostPower:   droneToChargePoint,
				}
				changePosition := PotentialTaskChange(taskPoints, chargePoints, node, beforeDutL, beforeDwtL, beforeDuchL, beforeDvchL)
				node.Weight = changePosition / math.Max(decidecount, 1)
				//if node.Weight != 0 {
				graph.AddNode(node)
				//}

				//graph.AddNode(node)
			}

		}
	}
	//6.汽车原地不动
	for _, car := range cars {
		node := GraphNode{
			NodeType: CarIdle,
			Car:      car,
			NodeKey:  fmt.Sprintf("<car%d>", car.Id),
			Weight:   softplus(0),
		}
		graph.AddNode(node)
		//初始化队列
		// 7汽车单独前往充电点的节点
		for _, chargePoint := range chargePoints {
			//if !hashmap[chargePoint.Id] {
			costTime := Distance(chargePoint.X, chargePoint.Y, car.X, car.Y) / car.CRget
			decidecount := math.Ceil((costTime) / float64(DecideTime)) //33 33/10=3

			node = GraphNode{
				NodeType:    CarToChargePoint,
				Car:         car,
				ChargePoint: chargePoint,
				Time:        (decidecount * float64(DecideTime)),
				NodeKey:     fmt.Sprintf("<car%d,c%d>", car.Id, chargePoint.Id),
				Weight:      0,
			}
			changePosition := PotentialTaskChange(taskPoints, chargePoints, node, beforeDutL, beforeDwtL, beforeDuchL, beforeDvchL)
			node.Weight = changePosition / math.Max(decidecount, 1)
			//if node.Weight != 0 {
			graph.AddNode(node)
			//}
			//graph.AddNode(node)
			//}
		}
	}
	//8.工作人员原地不动
	for _, worker := range workers {
		node := GraphNode{
			NodeType: WorkerIdle,
			Worker:   worker,
			NodeKey:  fmt.Sprintf("<w%d>", worker.Id),
			Weight:   softplus(0),
		}
		graph.AddNode(node)
		//初始化队列

		for _, taskPoint := range taskPoints {
			workerToTaskDist := Distance(worker.X, worker.Y, taskPoint.X, taskPoint.Y)
			workerToTaskDistTime := (workerToTaskDist / worker.WRget)
			decidecount := math.Ceil((workerToTaskDistTime) / float64(DecideTime)) //33 33/10=3
			// 9. 工作人员单独执行任务的节点
			//if decidecount > 1 {
			//	continue
			//}
			//WorkerChange := CalculateAvgDistanceChange(worker, taskPoint, taskPoints)
			node := GraphNode{
				NodeType:  WorkerToTask,
				Worker:    worker,
				TaskPoint: taskPoint,
				NodeKey:   fmt.Sprintf("<w%d,t%d>", worker.Id, taskPoint.Id),
				//Weight:    CalculateEfficiencyForWorkerToTask(float64((decidecount) * DecideTime)),
				Weight: 0,
				Time:   ((decidecount) * float64(DecideTime)),
			}
			changePosition := PotentialTaskChange(taskPoints, chargePoints, node, beforeDutL, beforeDwtL, beforeDuchL, beforeDvchL)
			node.Weight = changePosition / math.Max(decidecount, 1)
			graph.AddNode(node)
		}
	}
	// 添加边（根据冲突检测）
	nodes := graph.Nodes
	for i, node1 := range nodes {
		for j, node2 := range nodes {
			if i != j && isConflict(node1, node2) {
				graph.AddEdge(node1.NodeKey, node2.NodeKey)
			}
		}
	}
	return graph
}

func CalculateEfficiencyForWorkerIdle(worker *Worker) float64 {
	return 0
}

func CalculateEfficiencyForCarIdle(car *Car) float64 {
	return 0
}

func CalculateEfficiencyForDroneIdle(drone *Drone) float64 {
	return 0
}

// GetNeighbors 返回指定节点的邻居节点
func (g *Graph) GetNeighbors(nodeKey string) []string {
	neighbors := []string{}
	for neighbor := range g.Edges[nodeKey] {
		neighbors = append(neighbors, neighbor)
	}
	return neighbors
}

// RemoveNode 删除图中的一个节点及其所有相关边
func (g *Graph) RemoveNode(nodeKey string) {
	// 删除与节点相关的所有边
	for neighbor := range g.Edges[nodeKey] {
		delete(g.Edges[neighbor], nodeKey)
	}
	delete(g.Edges, nodeKey)
	// 删除节点本身
	delete(g.Nodes, nodeKey)
}

// getLossValue 计算给定节点的损失值
func (g *Graph) getLossValue(node *GraphNode) float64 {

	neighborWeight := 0.0
	for _, neighbor := range g.getNeighbors(node.NodeKey) {
		neighborWeight += g.Nodes[neighbor].Weight
	}
	return neighborWeight - node.Weight
}

// getNeighbors 获取指定节点的所有邻居
func (g *Graph) getNeighbors(nodeKey string) []string {
	neighbors := []string{}
	for neighbor := range g.Edges[nodeKey] {
		neighbors = append(neighbors, neighbor)
	}
	return neighbors
}

// isNeighbor 检查两个节点是否是邻居
func (g *Graph) isNeighbor(node1Key, node2Key string) bool {
	_, exists := g.Edges[node1Key][node2Key]
	return exists
}

// getCommonNeighbors 获取两个节点的公共邻居
func (g *Graph) getCommonNeighbors(node1Key, node2Key string) []string {
	neighbors1 := g.getNeighbors(node1Key)
	neighbors2 := g.getNeighbors(node2Key)

	commonNeighbors := []string{}
	neighborsMap := make(map[string]struct{})

	for _, neighbor := range neighbors1 {
		neighborsMap[neighbor] = struct{}{}
	}

	for _, neighbor := range neighbors2 {
		if _, exists := neighborsMap[neighbor]; exists {
			commonNeighbors = append(commonNeighbors, neighbor)
		}
	}

	return commonNeighbors
}

func (g *Graph) HILS(s1 *Solution, maxIterations int) (*Solution, bool) {
	start := time.Now()
	isChange := false

	// 4. 初始化解（使用处理后的图）
	s := NewSolution(g)
	for so := range s1.solution {
		s.addVertex(so)
	}

	// 继续原有的初始化
	for !s.isMaximal() {
		s.addRandomVertex()
		if !s.integrityCheck() {
			return nil, false
		}
	}

	// 4. 初始局部搜索
	iter := 0
	flag := true
	for flag {
		flag1 := s.omegaImprovement()
		if !(flag1) {
			break
		}
		for !s.isMaximal() {
			s.addRandomVertex()
		}
	}

	// 5. 记录最优解
	bestSolution := new(Solution)
	copySolution(s, bestSolution)
	localBestWeight := bestSolution.weight

	// 6. 主迭代循环参数
	maxUpdateIter := 0

	noImprovementCount := 0
	maxNoImprovementIters := maxIterations / 2.0
	// 7. 自适应参数
	perturbationStrength := 1
	acceptanceThreshold := 0.95 // 接受稍差解的阈值

	for iter < maxIterations && noImprovementCount < maxNoImprovementIters {
		iter++
		// 8. 创建候选解
		nextSolution := new(Solution)
		copySolution(s, nextSolution)

		// 改进后的扰动策略
		if noImprovementCount > maxNoImprovementIters/2 {
			// 大规模扰动阶段
			if s.weight > 0.95*s.g.BestKnownWeight {
				// 解集质量较高，但停滞不前
				perturbationStrength = 10 // 适中规模扰动
			} else {
				// 解质量较差，可能陷入局部最优,重新找解
				nextSolution = NewSolution(g)
				nextSolution.g = g
				for !nextSolution.isMaximal() {
					nextSolution.addRandomVertex()
				}
				perturbationStrength = 1
			}
		} else {
			// 小规模扰动阶段
			if s.weight > 0.9*s.g.BestKnownWeight {
				// 解质量较高，进行细致探索
				perturbationStrength = 1
			} else {
				// 解质量一般，增加探索范围
				perturbationStrength = 2
			}
		}

		// 10. 应用改进的扰动
		nextSolution.improvedForce(iter, perturbationStrength)

		// 11. 局部搜索
		for !nextSolution.isMaximal() {
			nextSolution.addRandomVertex()
		}

		flag := true
		for flag {
			flag1 := nextSolution.omegaImprovement()
			if !(flag1) {
				//flag = nextSolution.twoStarMoves()
				break
			}
			for !nextSolution.isMaximal() {
				nextSolution.addRandomVertex()
			}
		}

		// 12. 解的接受准则
		acceptNewSolution := false
		if nextSolution.weight > s.weight {
			acceptNewSolution = true
			noImprovementCount++
		} else if float64(nextSolution.weight)/float64(s.weight) > acceptanceThreshold {
			// 接受稍差的解以增加多样性
			acceptNewSolution = true
			noImprovementCount++
		} else {
			noImprovementCount++
		}
		//g.addSolutionToHistory(nextSolution)
		// 13. 更新当前解和最优解
		if acceptNewSolution {
			s = nextSolution
			//fa.updateFrequencies(s)

			if s.weight > localBestWeight {
				localBestWeight = s.weight
				isChange = true
				perturbationStrength = 1 // 重置扰动强度
			}

			if bestSolution.weight < s.weight {
				copySolution(s, bestSolution)
				maxUpdateIter = iter
				noImprovementCount = 0
				isChange = true
				g.BestKnownWeight = float64(s.weight) // 添加这行
			}
		}

		// 14. 动态调整参数
		if noImprovementCount > maxNoImprovementIters/4 {
			acceptanceThreshold *= 0.99 // 降低接受阈值
		}
	}

	fmt.Printf("最后更新迭代次数: %d\n", maxUpdateIter)
	fmt.Printf("最优解权重: %d\n", bestSolution.weight)
	g.BestKnownWeight = bestSolution.weight
	fmt.Printf("解中的节点: %v\n", bestSolution.solution)
	elapsed := time.Since(start).Seconds()
	println("这小轮耗时", elapsed, "s")
	return bestSolution, isChange
}

//	func (g *Graph) HILS() *Solution {
//		start := time.Now()
//		s := NewSolution(g)
//
//		// 随机初始化解
//		for !s.isMaximal() {
//			s.addRandomVertex()
//			if !s.integrityCheck() {
//				fmt.Println("Integrity check failed after adding random vertex")
//				return nil
//			}
//		}
//
//		flag := true
//		for flag {
//			flag1 := s.omegaImprovement()
//			//flag2 := s.AAPMoves()
//			if !(flag1) {
//				flag = s.twoImprovement()
//			}
//			for !s.isMaximal() {
//				s.addRandomVertex()
//			}
//		}
//
//		bestSolution := new(Solution)
//		copySolution(s, bestSolution)
//		localBestWeight := s.weight
//		iter := 0
//		maxUpdateIter := 0
//		p1 := 3
//		p2 := 4
//		p3 := 2 //扰乱程度
//		k := 1
//		maxIterations := 5000 // 根据需要设置
//		for iter < maxIterations {
//			iter++
//
//			// 深拷贝当前解
//			nextSolution := new(Solution)
//			copySolution(s, nextSolution)
//
//			// 抖动（shake）
//			nextSolution.force(1) // 参数根据需要设置
//
//			// 局部改进
//			for !nextSolution.isMaximal() {
//				nextSolution.addRandomVertex()
//			}
//			flag := true
//			for flag {
//				flag1 := nextSolution.omegaImprovement()
//				//flag2 := s.AAPMoves()
//				if !(flag1) {
//					//flag = nextSolution.twoStarMoves()
//					break
//				}
//				for !nextSolution.isMaximal() {
//					nextSolution.addRandomVertex()
//				}
//			}
//
//			if nextSolution.weight > s.weight {
//				k = 1
//				s = nextSolution
//				if nextSolution.weight > localBestWeight {
//					localBestWeight = nextSolution.weight
//					//bestSolution = s
//					k -= len(s.solution) / p1
//
//				}
//				if bestSolution.weight < s.weight {
//					copySolution(s, bestSolution)
//					k -= len(s.solution) * p2
//					maxUpdateIter = iter
//					//fmt.Printf("New best weight: %d at iteration %d\n", bestSolution.weight, iter)
//				}
//			} else if k <= len(s.solution)/p1 {
//				k = k + 1
//			} else {
//				localBestWeight = s.weight
//				s.force(p3)
//				k = 1
//			}
//			//println("iteration", iter)
//			//println(bestSolution.weight)
//		}
//		fmt.Printf("max Iter: %d\n", maxUpdateIter)
//		fmt.Printf("Best solution weight: %d\n", bestSolution.weight)
//		fmt.Printf("Solution nodes: %v\n", bestSolution.solution)
//		elapsed := time.Since(start).Seconds()
//		println("总耗时", elapsed, "s")
//		return bestSolution
//	}
func copySolution(s *Solution, nextSolution *Solution) {
	nextSolution.solution = make(map[string]bool)
	for k, v := range s.solution {
		nextSolution.solution[k] = v
	}
	nextSolution.tightness = make(map[string]int)
	for k, v := range s.tightness {
		nextSolution.tightness[k] = v
	}
	nextSolution.mu = make(map[string]float64)
	for k, v := range s.mu {
		nextSolution.mu[k] = v
	}
	nextSolution.freeNodes = append([]string{}, s.freeNodes...)
	nextSolution.nonFreeNodes = append([]string{}, s.nonFreeNodes...)
	nextSolution.weight = s.weight
	nextSolution.g = s.g
}
