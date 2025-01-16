package core

import (
	"fmt"
	"math"
	"time"
)

// Graph 结构体表示有向无权图
type Graph struct {
	Nodes map[string]*GraphNode      // 节点集合（组合 -> 权重）
	Edges map[string]map[string]bool // 边集合（节点 -> 其他相连节点的映射）
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
			Weight:   CalculateEfficiencyForDroneIdle(drone),
			NodeKey:  fmt.Sprintf("<u%d>", drone.Id),
		}
		graph.AddNode(node)
		//2. 无人机单独执行任务的情况
		for _, taskPoint := range taskPoints {
			droneToTaskDist := Distance(drone.X, drone.Y, taskPoint.X, taskPoint.Y)
			droneToTaskDistTime := (droneToTaskDist / drone.URget)
			decidecount := int(droneToTaskDistTime) / DecideTime //33 33/10=3
			taskCompleteDecideTime := (decidecount + 1) * DecideTime
			if drone.RemainingPower < droneToTaskDist+taskPoint.ChargeDist { //保证等到完成任务后的下一个决策时刻是可以续航的
				continue
			}
			//Dronechange := CalculateAvgDistanceChange(drone, taskPoint, taskPoints)
			//3.无人机和工作人员一起执行任务的节点
			for _, worker := range workers {
				workerToTaskDist := Distance(worker.X, worker.Y, taskPoint.X, taskPoint.Y)
				workerToTaskDistTime := (workerToTaskDist / worker.WRget)
				taskPointNeedTime := taskPoint.CostPow / drone.URget
				coaTaskTime := math.Max(workerToTaskDistTime, droneToTaskDistTime) + taskPointNeedTime
				decidecount = int(coaTaskTime) / DecideTime //33 33/10=3
				taskCompleteDecideTime = (decidecount + 1) * DecideTime
				if drone.RemainingPower < droneToTaskDist+taskPoint.ChargeDist+taskPoint.CostPow { //保证等到完成任务后的下一个决策时刻是可以续航的
					continue
				}

				//WorkerChange := CalculateAvgDistanceChange(worker, taskPoint, taskPoints)

				node := GraphNode{
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

				changePosition := PotentialTaskChange(taskPoints, chargePoints, drones, workers, cars, node, actions, index)
				node.Weight = (float64(changePosition) + 1*float64(len(taskPoints)+len(chargePoints))) / (float64(taskCompleteDecideTime + alpha))
				graph.AddNode(node)
			}
			if decidecount > 1 {
				continue
			}
			node := GraphNode{
				NodeType:  DroneToTask,
				Drone:     drone,
				TaskPoint: taskPoint,
				//Weight:    CalculateEfficiencyForDroneToTask(float64(taskCompleteDecideTime)),
				Weight:    0,
				NodeKey:   fmt.Sprintf("<u%d,t%d>", drone.Id, taskPoint.Id),
				Time:      float64(taskCompleteDecideTime),
				CostPower: droneToTaskDist,
			}
			changePosition := PotentialTaskChange(taskPoints, chargePoints, drones, workers, cars, node, actions, index)
			node.Weight = (float64(changePosition)) / float64(taskCompleteDecideTime+alpha)
			graph.AddNode(node)
		}

		for _, chargePoint := range chargePoints {
			droneToChargePoint := Distance(chargePoint.X, chargePoint.Y, drone.X, drone.Y)
			costTime := droneToChargePoint / drone.URget
			decidecount := int(costTime) / DecideTime      //33 33/10=3
			if drone.RemainingPower < droneToChargePoint { //无人机无法到达该充电点
				continue
			}
			// 5. 无人机和汽车一起前往充电点的节点
			for _, car := range cars {
				carToChargePoint := Distance(chargePoint.X, chargePoint.Y, car.X, car.Y) / car.CRget
				coaTaskTime := math.Max(carToChargePoint, costTime) + drone.FullPower/car.ChargePow
				decidecount := int(coaTaskTime)/DecideTime + 1 //33 33/10=3

				node := GraphNode{
					NodeType:    DroneCarToChargePoint,
					Drone:       drone,
					Car:         car,
					ChargePoint: chargePoint,
					Time:        float64(decidecount * DecideTime),
					Weight:      0,
					NodeKey:     fmt.Sprintf("<u%d,car%d,c%d>", drone.Id, car.Id, chargePoint.Id),
				}
				changePosition := PotentialTaskChange(taskPoints, chargePoints, drones, workers, cars, node, actions, index)
				node.Weight = (float64(changePosition)) / float64(decidecount*DecideTime+alpha)
				graph.AddNode(node)
				hashmap[chargePoint.Id] = true
			}
			// 4. 无人机单独前往充电点的节点
			if !hashmap[chargePoint.Id] { //这个充电点没有被任何<u1,c1,car1>分配
				node := GraphNode{
					NodeType:    DroneToChargePoint,
					Drone:       drone,
					ChargePoint: chargePoint,
					Time:        float64(decidecount * DecideTime),
					Weight:      0,
					NodeKey:     fmt.Sprintf("<u%d,c%d>", drone.Id, chargePoint.Id),
					CostPower:   droneToChargePoint,
				}
				changePosition := PotentialTaskChange(taskPoints, chargePoints, drones, workers, cars, node, actions, index)
				node.Weight = (float64(changePosition)) / float64(decidecount*DecideTime+alpha)
				graph.AddNode(node)
			}

		}
	}
	//6.汽车原地不动
	for _, car := range cars {
		node := GraphNode{
			NodeType: CarIdle,
			Car:      car,
			NodeKey:  fmt.Sprintf("<car%d>", car.Id),
			Weight:   CalculateEfficiencyForCarIdle(car),
		}
		graph.AddNode(node)

		// 7汽车单独前往充电点的节点
		for _, chargePoint := range chargePoints {
			//if !hashmap[chargePoint.Id] {
			costTime := Distance(chargePoint.X, chargePoint.Y, car.X, car.Y) / car.CRget
			decidecount := int(costTime)/DecideTime + 1 //33 33/10=3
			//if decidecount > 2 {
			//	continue
			//}
			node := GraphNode{
				NodeType:    CarToChargePoint,
				Car:         car,
				ChargePoint: chargePoint,
				Time:        float64(decidecount * DecideTime),
				NodeKey:     fmt.Sprintf("<car%d,c%d>", car.Id, chargePoint.Id),
				Weight:      0,
			}
			changePosition := PotentialTaskChange(taskPoints, chargePoints, drones, workers, cars, node, actions, index)
			node.Weight = (float64(changePosition)) / float64(decidecount*DecideTime+alpha)
			graph.AddNode(node)
			//}
		}
	}
	//8.工作人员原地不动
	for _, worker := range workers {
		node := GraphNode{
			NodeType: WorkerIdle,
			Worker:   worker,
			NodeKey:  fmt.Sprintf("<w%d>", worker.Id),
			Weight:   CalculateEfficiencyForWorkerIdle(worker),
		}
		graph.AddNode(node)

		for _, taskPoint := range taskPoints {
			workerToTaskDist := Distance(worker.X, worker.Y, taskPoint.X, taskPoint.Y)
			workerToTaskDistTime := (workerToTaskDist / worker.WRget)
			decidecount := int(workerToTaskDistTime)/DecideTime + 1 //33 33/10=3
			// 9. 工作人员单独执行任务的节点
			if decidecount > 1 {
				continue
			}
			//WorkerChange := CalculateAvgDistanceChange(worker, taskPoint, taskPoints)
			node := GraphNode{
				NodeType:  WorkerToTask,
				Worker:    worker,
				TaskPoint: taskPoint,
				NodeKey:   fmt.Sprintf("<w%d,t%d>", worker.Id, taskPoint.Id),
				//Weight:    CalculateEfficiencyForWorkerToTask(float64((decidecount) * DecideTime)),
				Weight: 0,
				Time:   float64((decidecount) * DecideTime),
			}
			changePosition := PotentialTaskChange(taskPoints, chargePoints, drones, workers, cars, node, actions, index)
			node.Weight = (float64(changePosition)) / float64(decidecount*DecideTime+alpha)
			graph.AddNode(node)
		}

		//for _, chargePoint := range chargePoints {
		//	workerToTaskDist := Distance(worker.X, worker.Y, chargePoint.X, chargePoint.Y)
		//	workerToTaskDistTime := (workerToTaskDist / worker.WRget)
		//	decidecount := int(workerToTaskDistTime)/DecideTime + 1 //33 33/10=3
		//	if decidecount > 1 {
		//		continue
		//	}
		//	node := GraphNode{
		//		NodeType:    WorkerToCharge,
		//		Worker:      worker,
		//		ChargePoint: chargePoint,
		//		NodeKey:     fmt.Sprintf("<w%d,c%d>", worker.Id, chargePoint.Id),
		//		//Weight:    CalculateEfficiencyForWorkerToTask(float64((decidecount) * DecideTime)),
		//		Weight: 0,
		//		Time:   float64((decidecount) * DecideTime),
		//	}
		//	changePosition := PotentialTaskChange(taskPoints, drones, workers, node)
		//	node.Weight = (float64(changePosition)) / float64(decidecount*DecideTime+alpha)
		//	graph.AddNode(node)
		//}
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

func (g *Graph) HILS() *Solution {
	start := time.Now()
	s := NewSolution(g)

	// 随机初始化解
	for !s.isMaximal() {
		s.addRandomVertex()
		if !s.integrityCheck() {
			fmt.Println("Integrity check failed after adding random vertex")
			return nil
		}
	}

	flag := true
	for flag {
		flag1 := s.omegaImprovement()
		//flag2 := s.AAPMoves()
		if !(flag1) {
			flag = s.twoImprovement()
		}
		for !s.isMaximal() {
			s.addRandomVertex()
		}
	}

	bestSolution := new(Solution)
	copySolution(s, bestSolution)
	localBestWeight := s.weight
	iter := 0
	maxUpdateIter := 0
	p1 := 3
	p2 := 4
	p3 := 2 //扰乱程度
	k := 1
	maxIterations := 5000 // 根据需要设置
	for iter < maxIterations {
		iter++

		// 深拷贝当前解
		nextSolution := new(Solution)
		copySolution(s, nextSolution)

		// 抖动（shake）
		nextSolution.force(1) // 参数根据需要设置

		// 局部改进
		for !nextSolution.isMaximal() {
			nextSolution.addRandomVertex()
		}
		flag := true
		for flag {
			flag1 := nextSolution.omegaImprovement()
			//flag2 := s.AAPMoves()
			if !(flag1) {
				flag = nextSolution.twoImprovement()
			}
			for !nextSolution.isMaximal() {
				nextSolution.addRandomVertex()
			}
		}

		if nextSolution.weight > s.weight {
			k = 1
			s = nextSolution
			if nextSolution.weight > localBestWeight {
				localBestWeight = nextSolution.weight
				//bestSolution = s
				k -= len(s.solution) / p1

			}
			if bestSolution.weight < s.weight {
				copySolution(s, bestSolution)
				k -= len(s.solution) * p2
				maxUpdateIter = iter
				//fmt.Printf("New best weight: %d at iteration %d\n", bestSolution.weight, iter)
			}
		} else if k <= len(s.solution)/p1 {
			k = k + 1
		} else {
			localBestWeight = s.weight
			s.force(p3)
			k = 1
		}

	}
	fmt.Printf("max Iter: %d\n", maxUpdateIter)
	fmt.Printf("Best solution weight: %d\n", bestSolution.weight)
	fmt.Printf("Solution nodes: %v\n", bestSolution.solution)
	elapsed := time.Since(start).Seconds()
	println("总耗时", elapsed, "s")
	return bestSolution
}
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

//第 1 天
//以下第 1 轮决策：
//max Iter: 811
//Best solution weight: %!d(float64=4.9249999999999945)
//Solution nodes: map[<car13,c17>:true <car17>:true <car4,c13>:true <car5>:true <u10,w1,t36>:true <u20,w40,t0>:true <u4,w7,t40>:true]
//10 无人机与 1 工作人员结成了联盟，完成任务 36 消耗时间为 +4.000000e+001 消耗电量 +2.284943e+001 权值 +1.300000e+000
//4 无人机与 7 工作人员结成了联盟，完成任务 40 消耗时间为 +8.000000e+001 消耗电量 +1.572792e+001 权值 +7.250000e-001
//20 无人机与 40 工作人员结成了联盟，完成任务 0 消耗时间为 +2.000000e+001 消耗电量 +9.403124e+000 权值 +2.900000e+000
//以下第 2 轮决策：
//max Iter: 211
//Best solution weight: %!d(float64=10.199999999999998)
//Solution nodes: map[<car0,c13>:true <car13>:true <car17>:true <car19>:true <car4,c3>:true <car5>:true <car6,c1>:true <car7,c8>:true <u11,w8,t7>:true <u21,w10,t53>:true <u23,w3,t19>:true <u24,w29,t42>:true]
//21 无人机与 10 工作人员结成了联盟，完成任务 53 消耗时间为 +2.000000e+001 消耗电量 +1.603840e+001 权值 +2.650000e+000
//24 无人机与 29 工作人员结成了联盟，完成任务 42 消耗时间为 +2.000000e+001 消耗电量 +1.028011e+001 权值 +2.650000e+000
//11 无人机与 8 工作人员结成了联盟，完成任务 7 消耗时间为 +2.000000e+001 消耗电量 +1.634166e+001 权值 +2.300000e+000
//23 无人机与 3 工作人员结成了联盟，完成任务 19 消耗时间为 +2.000000e+001 消耗电量 +7.123106e+000 权值 +2.600000e+000
//以下第 3 轮决策：
//max Iter: 902
//Best solution weight: %!d(float64=7.550000000000009)
//Solution nodes: map[<car0,c18>:true <car1,c4>:true <car13,c17>:true <car16>:true <car17,c10>:true <car19>:true <car4,c3>:true <car5>:true <car6,c1>:true <car7,c8>:true <u20,w13,t51>:true <u22,w26,t43>:true <u28,w43,t56>:true <u8,w40,t32>:true]
//8 无人机与 40 工作人员结成了联盟，完成任务 32 消耗时间为 +2.000000e+001 消耗电量 +5.236068e+000 权值 +2.700000e+000
//22 无人机与 26 工作人员结成了联盟，完成任务 43 消耗时间为 +2.000000e+001 消耗电量 +1.440175e+001 权值 +2.650000e+000
//28 无人机与 43 工作人员结成了联盟，完成任务 56 消耗时间为 +1.000000e+002 消耗电量 +1.329563e+001 权值 +5.000000e-001
//20 无人机与 13 工作人员结成了联盟，完成任务 51 消耗时间为 +3.000000e+001 消耗电量 +1.418034e+001 权值 +1.700000e+000
//以下第 4 轮决策：
//max Iter: 527
//Best solution weight: %!d(float64=8.149999999999999)
//Solution nodes: map[<car0,c13>:true <car13>:true <car15,c15>:true <car16>:true <car17,c4>:true <car19,c5>:true <car4,c1>:true <car5>:true <car7,c8>:true <u11,car1,c7>:true <u19,w14,t33>:true <u21,car6,c11>:true <u23,w3,t6>:true <u24,car1,c7>:true <w10>:true <w19>:true <w27>:true <w29>:true <w8>:true <w9>:true]
//汽车 1 在充电点 7 开始去对无人机 24 的充电，消耗时间 +2.000000e+001
//23 无人机与 3 工作人员结成了联盟，完成任务 6 消耗时间为 +1.000000e+001 消耗电量 +4.000000e+000 权值 +4.700000e+000
//汽车 6 在充电点 11 开始去对无人机 21 的充电，消耗时间 +2.000000e+001
//汽车 1 在充电点 7 开始去对无人机 11 的充电，消耗时间 +2.000000e+001
//19 无人机与 14 工作人员结成了联盟，完成任务 33 消耗时间为 +2.000000e+001 消耗电量 +9.324555e+000 权值 +2.200000e+000
//以下第 5 轮决策：
//max Iter: 428
//Best solution weight: %!d(float64=15.05)
//Solution nodes: map[<car0,c13>:true <car15,c15>:true <car16>:true <car19,c5>:true <car2>:true <car4,c1>:true <car5>:true <car7>:true <u10,car17,c14>:true <u15,w19,t59>:true <u18,w26,t5>:true <u22,car13,c17>:true <u23,car17,c14>:true <u26,w16,t25>:true <u7,w8,t50>:true <u8,w29,t11>:true <u9,w23,t35>:true <w0>:true <w10>:true <w1>:true <w27>:true <w36,t4>:true <w3>:true <w40>:true <w45,t44>:true <w9>:true]
//18 无人机与 26 工作人员结成了联盟，完成任务 5 消耗时间为 +3.000000e+001 消耗电量 +4.000000e+000 权值 +1.566667e+000
//26 无人机与 16 工作人员结成了联盟，完成任务 25 消耗时间为 +2.000000e+001 消耗电量 +7.123106e+000 权值 +2.150000e+000
//7 无人机与 8 工作人员结成了联盟，完成任务 50 消耗时间为 +2.000000e+001 消耗电量 +1.000000e+001 权值 +2.250000e+000
//汽车 13 在充电点 17 开始去对无人机 22 的充电，消耗时间 +1.000000e+001
//8 无人机与 29 工作人员结成了联盟，完成任务 11 消耗时间为 +1.000000e+001 消耗电量 +8.000000e+000 权值 +4.200000e+000
//9 无人机与 23 工作人员结成了联盟，完成任务 35 消耗时间为 +2.000000e+001 消耗电量 +1.028011e+001 权值 +2.200000e+000
//15 无人机与 19 工作人员结成了联盟，完成任务 59 消耗时间为 +3.000000e+001 消耗电量 +1.862050e+001 权值 +1.433333e+000
//汽车 17 在充电点 14 开始去对无人机 10 的充电，消耗时间 +2.000000e+001
//汽车 17 在充电点 14 开始去对无人机 23 的充电，消耗时间 +2.000000e+001
//以下第 6 轮决策：
//max Iter: 300
//Best solution weight: %!d(float64=14.603333333333335)
//Solution nodes: map[<car0>:true <car13,c17>:true <car14,c2>:true <car15>:true <car16,c5>:true <car19>:true <car1>:true <car2>:true <car4>:true <car5>:true <car6,c11>:true <car7,c8>:true <u11,w40,t58>:true <u12,w45,t44>:true <u14,w13,t54>:true <u19,w14,t3>:true <u20,car3,c13>:true <u21,w1,t27>:true <u22,w49,t9>:true <u24,w44,t12>:true <u25,w0,t55>:true <u8,w10,t28>:true <w24>:true <w27>:true <w28>:true <w29>:true <w36,t4>:true <w3>:true <w46>:true <w6,t31>:true <w9>:true]
//21 无人机与 1 工作人员结成了联盟，完成任务 27 消耗时间为 +3.000000e+001 消耗电量 +1.964332e+001 权值 +1.400000e+000
//8 无人机与 10 工作人员结成了联盟，完成任务 28 消耗时间为 +5.000000e+001 消耗电量 +8.000000e+000 权值 +8.200000e-001
//22 无人机与 49 工作人员结成了联盟，完成任务 9 消耗时间为 +4.000000e+001 消耗电量 +1.081025e+001 权值 +1.000000e+000
//11 无人机与 40 工作人员结成了联盟，完成任务 58 消耗时间为 +3.000000e+001 消耗电量 +1.205539e+001 权值 +1.333333e+000
//汽车 3 在充电点 13 开始去对无人机 20 的充电，消耗时间 +2.000000e+001
//25 无人机与 0 工作人员结成了联盟，完成任务 55 消耗时间为 +3.000000e+001 消耗电量 +9.082763e+000 权值 +1.366667e+000
//14 无人机与 13 工作人员结成了联盟，完成任务 54 消耗时间为 +2.000000e+001 消耗电量 +8.656854e+000 权值 +1.950000e+000
//19 无人机与 14 工作人员结成了联盟，完成任务 3 消耗时间为 +3.000000e+001 消耗电量 +9.000000e+000 权值 +1.366667e+000
//24 无人机与 44 工作人员结成了联盟，完成任务 12 消耗时间为 +3.000000e+001 消耗电量 +1.776482e+001 权值 +1.366667e+000
//12 无人机与 45 工作人员结成了联盟，完成任务 44 消耗时间为 +1.000000e+001 消耗电量 +8.099020e+000 权值 +4.000000e+000
//以下第 7 轮决策：
//max Iter: 862
//Best solution weight: %!d(float64=12.250000000000002)
//Solution nodes: map[<car12,c5>:true <car15>:true <car16>:true <car19>:true <car1>:true <car2>:true <car6>:true <car7>:true <u1,w20,t52>:true <u12,w3,t57>:true <u16,w45,t41>:true <u2,w8,t45>:true <u23,w9,t2>:true <u26,w33,t46>:true <u5,w16,t23>:true <u6,w36,t4>:true <u7,car14,c19>:true <u9,car0,c18>:true <w17>:true <w23>:true <w24>:true <w27>:true <w28>:true <w29>:true <w41>:true <w46>:true <w47>:true <w6>:true]
//6 无人机与 36 工作人员结成了联盟，完成任务 4 消耗时间为 +2.000000e+001 消耗电量 +1.221954e+001 权值 +1.600000e+000
//12 无人机与 3 工作人员结成了联盟，完成任务 57 消耗时间为 +2.000000e+001 消耗电量 +7.123106e+000 权值 +1.600000e+000
//1 无人机与 20 工作人员结成了联盟，完成任务 52 消耗时间为 +3.000000e+001 消耗电量 +1.000000e+001 权值 +1.066667e+000
//5 无人机与 16 工作人员结成了联盟，完成任务 23 消耗时间为 +1.000000e+001 消耗电量 +7.123106e+000 权值 +3.200000e+000
//16 无人机与 45 工作人员结成了联盟，完成任务 41 消耗时间为 +2.000000e+001 消耗电量 +8.099020e+000 权值 +1.600000e+000
//汽车 0 在充电点 18 开始去对无人机 9 的充电，消耗时间 +2.000000e+001
//汽车 14 在充电点 19 开始去对无人机 7 的充电，消耗时间 +3.000000e+001
//2 无人机与 8 工作人员结成了联盟，完成任务 45 消耗时间为 +2.000000e+001 消耗电量 +1.300000e+001 权值 +1.450000e+000
//23 无人机与 9 工作人员结成了联盟，完成任务 2 消耗时间为 +3.000000e+001 消耗电量 +1.194427e+001 权值 +9.333333e-001
//26 无人机与 33 工作人员结成了联盟，完成任务 46 消耗时间为 +4.000000e+001 消耗电量 +1.100000e+001 权值 +8.000000e-001
//以下第 8 轮决策：
//max Iter: 232
//Best solution weight: %!d(float64=4.75)
//Solution nodes: map[<car12>:true <car15>:true <car16,c17>:true <car2>:true <car3,c13>:true <u14,w23,t47>:true <u15,car1,c6>:true <u18,w26,t17>:true <u29,w47,t20>:true <u5,w16,t10>:true <w12>:true <w13>:true <w17>:true <w19>:true <w2,t20>:true <w24>:true <w27>:true <w28>:true <w41>:true <w46>:true <w6>:true]
//5 无人机与 16 工作人员结成了联盟，完成任务 10 消耗时间为 +2.000000e+001 消耗电量 +5.000000e+000 权值 +1.200000e+000
//18 无人机与 26 工作人员结成了联盟，完成任务 17 消耗时间为 +3.000000e+001 消耗电量 +8.000000e+000 权值 +8.000000e-001
//汽车 1 在充电点 6 开始去对无人机 15 的充电，消耗时间 +1.000000e+001
//29 无人机与 47 工作人员结成了联盟，完成任务 20 消耗时间为 +2.000000e+001 消耗电量 +1.363015e+001 权值 +1.100000e+000
//14 无人机与 23 工作人员结成了联盟，完成任务 47 消耗时间为 +2.000000e+001 消耗电量 +5.828427e+000 权值 +1.050000e+000
//以下第 9 轮决策：
//max Iter: 777
//Best solution weight: %!d(float64=3.933333333333336)
//Solution nodes: map[<car12>:true <car15>:true <u12,car3,c2>:true <u15,w45,t26>:true <u16,car3,c2>:true <u17,w2,t39>:true <u19,car10,c14>:true <u2,car3,c2>:true <u25,w0,t34>:true <u6,car2,c3>:true <u9,w36,t22>:true <w12,t1>:true <w14>:true <w17>:true <w19>:true <w21>:true <w24>:true <w27>:true <w28>:true <w38>:true <w41>:true <w44>:true <w46>:true <w6>:true]
//9 无人机与 36 工作人员结成了联盟，完成任务 22 消耗时间为 +3.000000e+001 消耗电量 +5.000000e+000 权值 +6.333333e-001
//汽车 3 在充电点 2 开始去对无人机 16 的充电，消耗时间 +2.000000e+001
//17 无人机与 2 工作人员结成了联盟，完成任务 39 消耗时间为 +2.000000e+001 消耗电量 +7.472136e+000 权值 +9.500000e-001
//15 无人机与 45 工作人员结成了联盟，完成任务 26 消耗时间为 +2.000000e+001 消耗电量 +1.124621e+001 权值 +9.000000e-001
//汽车 3 在充电点 2 开始去对无人机 2 的充电，消耗时间 +2.000000e+001
//汽车 10 在充电点 14 开始去对无人机 19 的充电，消耗时间 +2.000000e+001
//汽车 3 在充电点 2 开始去对无人机 12 的充电，消耗时间 +2.000000e+001
//25 无人机与 0 工作人员结成了联盟，完成任务 34 消耗时间为 +2.000000e+001 消耗电量 +6.162278e+000 权值 +1.000000e+000
//汽车 2 在充电点 3 开始去对无人机 6 的充电，消耗时间 +1.000000e+001
//以下第 10 轮决策：
//max Iter: 168
//Best solution weight: %!d(float64=3.7666666666666666)
//Solution nodes: map[<car11,c3>:true <car14,c12>:true <car2,c16>:true <car8>:true <u0,w35,t8>:true <u1,w47,t37>:true <u14,w46,t13>:true <u29,w20,t15>:true <u5,car12,c17>:true <u6,w23,t49>:true <u7,w42,t38>:true <w12,t1>:true <w16>:true <w17>:true <w21>:true <w24>:true <w28>:true <w30>:true <w34>:true <w37>:true <w38>:true <w41>:true <w44>:true <w49>:true <w5>:true <w6>:true]
//7 无人机与 42 工作人员结成了联盟，完成任务 38 消耗时间为 +4.000000e+001 消耗电量 +9.403124e+000 权值 +4.000000e-001
//0 无人机与 35 工作人员结成了联盟，完成任务 8 消耗时间为 +3.000000e+001 消耗电量 +1.021110e+001 权值 +5.333333e-001
//14 无人机与 46 工作人员结成了联盟，完成任务 13 消耗时间为 +3.000000e+001 消耗电量 +9.324555e+000 权值 +5.000000e-001
//29 无人机与 20 工作人员结成了联盟，完成任务 15 消耗时间为 +2.000000e+001 消耗电量 +8.656854e+000 权值 +8.000000e-001
//6 无人机与 23 工作人员结成了联盟，完成任务 49 消耗时间为 +3.000000e+001 消耗电量 +1.508305e+001 权值 +5.333333e-001
//1 无人机与 47 工作人员结成了联盟，完成任务 37 消耗时间为 +2.000000e+001 消耗电量 +5.828427e+000 权值 +8.000000e-001
//汽车 12 在充电点 17 开始去对无人机 5 的充电，消耗时间 +1.000000e+001
//以下第 11 轮决策：
//max Iter: 372
//Best solution weight: %!d(float64=2.55)
//Solution nodes: map[<car10>:true <car11,c1>:true <car12,c5>:true <car14,c0>:true <car3,c4>:true <car8>:true <car9>:true <u12,w38,t21>:true <u16,w12,t1>:true <u17,w2,t24>:true <u2,w34,t29>:true <u25>:true <u5,w6,t31>:true <w15>:true <w17>:true <w18>:true <w21>:true <w24>:true <w28>:true <w30>:true <w33>:true <w37>:true <w39>:true <w41>:true <w44>:true <w49>:true <w5>:true]
//16 无人机与 12 工作人员结成了联盟，完成任务 1 消耗时间为 +2.000000e+001 消耗电量 +1.660147e+001 权值 +4.000000e-001
//12 无人机与 38 工作人员结成了联盟，完成任务 21 消耗时间为 +2.000000e+001 消耗电量 +1.154400e+001 权值 +4.500000e-001
//17 无人机与 2 工作人员结成了联盟，完成任务 24 消耗时间为 +4.000000e+001 消耗电量 +1.205539e+001 权值 +2.500000e-001
//2 无人机与 34 工作人员结成了联盟，完成任务 29 消耗时间为 +2.000000e+001 消耗电量 +1.536932e+001 权值 +4.500000e-001
//5 无人机与 6 工作人员结成了联盟，完成任务 31 消耗时间为 +1.000000e+001 消耗电量 +8.000000e+000 权值 +1.000000e+000
//以下第 12 轮决策：
//max Iter: 140
//Best solution weight: %!d(float64=0.45000000000000007)
//Solution nodes: map[<car10>:true <car12,c17>:true <car18,c2>:true <car8>:true <car9,c0>:true <u1,car11,c1>:true <u13,w11,t48>:true <u27>:true <u29,car11,c1>:true <u3>:true <u5>:true <w15>:true <w17>:true <w18>:true <w20>:true <w21>:true <w22>:true <w25>:true <w30>:true <w31>:true <w32>:true <w33>:true <w37>:true <w39>:true <w41>:true <w47>:true <w48>:true <w4>:true <w5>:true]
//汽车 11 在充电点 1 开始去对无人机 1 的充电，消耗时间 +1.000000e+001
//13 无人机与 11 工作人员结成了联盟，完成任务 48 消耗时间为 +2.000000e+001 消耗电量 +1.634166e+001 权值 +2.500000e-001
//汽车 11 在充电点 1 开始去对无人机 29 的充电，消耗时间 +1.000000e+001
//以下第 13 轮决策：
//max Iter: 5
//Best solution weight: %!d(float64=0.2)
//Solution nodes: map[<car10>:true <car11>:true <car18,c4>:true <car8>:true <car9,c19>:true <u0,w35,t30>:true <u27>:true <u29>:true <u3>:true <w12>:true <w15>:true <w18>:true <w21>:true <w22>:true <w25>:true <w30>:true <w31>:true <w32>:true <w34>:true <w37,t14>:true <w38>:true <w39>:true <w48>:true <w4>:true <w5>:true]
//0 无人机与 35 工作人员结成了联盟，完成任务 30 消耗时间为 +2.000000e+001 消耗电量 +6.162278e+000 权值 +2.000000e-001
//以下第 14 轮决策：
//max Iter: 0
//Best solution weight: %!d(float64=0)
//Solution nodes: map[<car10,c4>:true <car18,c14>:true <car8>:true <car9,c12>:true <u13,car11,c5>:true <u27>:true <u3>:true <w11>:true <w15>:true <w18>:true <w21>:true <w22>:true <w25>:true <w30>:true <w31>:true <w32>:true <w34>:true <w37,t14>:true <w38>:true <w39>:true <w42>:true <w48>:true <w4>:true <w5>:true]
//汽车 11 在充电点 5 开始去对无人机 13 的充电，消耗时间 +5.000000e+001
//以下第 15 轮决策：
//max Iter: 0
//Best solution weight: %!d(float64=0)
//Solution nodes: map[<car8>:true <car9,c0>:true <u0,car18,c18>:true <u27>:true <u3>:true <w11>:true <w15>:true <w18>:true <w22>:true <w25>:true <w30>:true <w31>:true <w32>:true <w34>:true <w35>:true <w37>:true <w39>:true <w42>:true <w48>:true <w4>:true <w5>:true]
//汽车 18 在充电点 18 开始去对无人机 0 的充电，消耗时间 +4.000000e+001
//以下第 16 轮决策：
//max Iter: 0
//Best solution weight: %!d(float64=0)
//Solution nodes: map[<car9,c12>:true <u27>:true <u3>:true <w11>:true <w15>:true <w18>:true <w22>:true <w25>:true <w31>:true <w32>:true <w39>:true <w48>:true <w4>:true]
//以下第 17 轮决策：
//max Iter: 0
//Best solution weight: %!d(float64=0)
//Solution nodes: map[<u27>:true <u3>:true <w11>:true <w22>:true <w25>:true <w31>:true <w32>:true <w48>:true <w4>:true]
//以下第 18 轮决策：
//max Iter: 0
//Best solution weight: %!d(float64=0)
//Solution nodes: map[]
//任务完成数 57
//完成率为 +9.500000e-001
