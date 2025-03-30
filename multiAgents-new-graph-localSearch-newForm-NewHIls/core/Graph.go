package core

import (
	"fmt"
	"math"
	"sort"
	"strconv"
	"strings"
	"time"
)

var count int

// Graph 结构体表示有向无权图
type Graph struct {
	Nodes           map[string]*GraphNode      // 节点集合（组合 -> 权重）
	Edges           map[string]map[string]bool // 边集合（节点 -> 其他相连节点的映射）
	NodeQueues      map[string]*NodeQueue
	QueuesSolutions QueuesSolutions
	SolutionHashSet map[string]bool
	BestKnownWeight float64 // 添加最优解权重追踪
}

// 频率分析结构体
type FrequencyAnalyzer struct {
	nodeFreq    map[string]float64
	edgeFreq    map[string]map[string]float64
	windowSize  int
	decayFactor float64
	iterations  [][]string // 存储最近W次迭代的解
}

// NewGraph 初始化图
func NewGraph() *Graph {
	return &Graph{
		Nodes:           make(map[string]*GraphNode),
		Edges:           make(map[string]map[string]bool),
		SolutionHashSet: map[string]bool{},
		NodeQueues:      make(map[string]*NodeQueue),
	}
}

// 添加节点
func (g *Graph) AddNode(UUID string, node GraphNode) {
	g.Nodes[node.NodeKey] = &node
	if g.NodeQueues[UUID] == nil {
		g.NodeQueues[UUID] = &NodeQueue{}
	}

	g.NodeQueues[UUID].Push(node)

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

func IsConflict(node1, node2 *GraphNode) bool {
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
		graph.AddNode(drone.GetUUId(), node)
		//初始化队列

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
				graph.AddNode(drone.UUID, node)
			}
			//if decidecount > 1 {
			//	continue
			//}
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
			//if node.Weight != 0 {
			graph.AddNode(drone.UUID, node)
			//}

			//graph.AddNode(node)
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
				graph.AddNode(drone.UUID, node)
				//graph.AddNode(node)
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
				//if node.Weight != 0 {
				graph.AddNode(drone.UUID, node)
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
			Weight:   CalculateEfficiencyForCarIdle(car),
		}
		graph.AddNode(car.UUID, node)
		//初始化队列
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
			//if node.Weight != 0 {
			graph.AddNode(car.UUID, node)
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
			Weight:   CalculateEfficiencyForWorkerIdle(worker),
		}
		graph.AddNode(worker.UUID, node)
		//初始化队列

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
			//if node.Weight != 0 {
			graph.AddNode(worker.UUID, node)
			//}
			//graph.AddNode(node)
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
			if i != j && IsConflict(node1, node2) {
				graph.AddEdge(node1.NodeKey, node2.NodeKey)
			}
		}
	}
	//for _, node1 := range nodes {
	//	node1.Weight = node1.Weight / float64(len(graph.Edges[node1.NodeKey]))
	//}
	//degree := 0
	//for _, node := range graph.Edges {
	//	degree += len(node)
	//
	//}
	//mean := degree / len(graph.Edges)
	//sum := 0.0
	//for _, node := range graph.Edges {
	//	sum += (float64(len(node) - mean)) * (float64(len(node) - mean))
	//}
	//variance := sum / float64(len(graph.Edges))
	//variance = math.Sqrt(variance)
	//println(mean)
	//println(variance)
	//for _, node := range graph.Nodes {
	//	print(node.NodeKey, ":", node.Weight, ",")
	//}

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

func (g *Graph) HILS(s1 *Solution) (*Solution, bool) {
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

	flag := true
	for flag {
		flag1 := s.omegaImprovement()
		//flag2 := s.AAPMoves()
		//flag3 := s.oneStarMoves()
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
	iter := 0
	maxUpdateIter := 0
	maxNoImprovementIters := 10000
	noImprovementCount := 0
	maxIterations := 1000000

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
		if nextSolution.weight > s.weight && !g.isVisited(nextSolution) {
			acceptNewSolution = true
			noImprovementCount++
		} else if float64(nextSolution.weight)/float64(s.weight) > acceptanceThreshold && !g.isVisited(nextSolution) {
			// 接受稍差的解以增加多样性
			acceptNewSolution = true
			noImprovementCount++
		} else {
			noImprovementCount++
		}
		g.addSolutionToHistory(nextSolution)
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

	//g = processedGraph
	//bestSolution.g = processedGraph
	//for nodeKey, node := range MustSolution {
	//	bestSolution.addVertex(nodeKey)
	//	bestSolution.weight = node.Weight
	//}
	fmt.Printf("最后更新迭代次数: %d\n", maxUpdateIter)
	fmt.Printf("最优解权重: %d\n", bestSolution.weight)
	g.BestKnownWeight = bestSolution.weight
	fmt.Printf("解中的节点: %v\n", bestSolution.solution)
	elapsed := time.Since(start).Seconds()
	println("这小轮耗时", elapsed, "s")
	return bestSolution, isChange
}

func (g *Graph) isVisited(solution *Solution) bool {
	if g.SolutionHashSet != nil {
		b := g.SolutionHashSet[solution.hash()]
		if b {
			count++
		}
		return b
	}
	return false
}

func (g *Graph) addSolutionToHistory(solution *Solution) {
	if g.SolutionHashSet == nil {
		g.SolutionHashSet = map[string]bool{}
	}
	g.SolutionHashSet[solution.hash()] = true
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
	nextSolution.SPlus = s.SPlus
	nextSolution.S2 = s.S2
	nextSolution.S1 = s.S1
	nextSolution.twoTightNeighborhood = s.twoTightNeighborhood
	nextSolution.tightNeighborhood = s.tightNeighborhood
	nextSolution.mates = s.mates
	nextSolution.g = s.g
}

// 图预处理函数
func (g *Graph) preprocess(count int) (map[string]struct{}, map[string]struct{}, bool) {
	progress := false
	removedNodes := make(map[string]struct{})
	solutoinNodes := make(map[string]struct{})

	reductions := []Reduction{
		&NeighborhoodReduction{},
		&DominanceReduction{}, // DominationReduction 实现了 Reduction 接口
		//&CliqueReduction{},    // TwinReduction 实现了 Reduction 接口
		&CliqueNeighborhoodReductionFast{},
	}
	for _, reduction := range reductions {
		reduction.Init()
	}

	// 遍历所有约简规则
	for _, reduction := range reductions {
		if count == 1 {
			for _, node := range g.Nodes {
				reduction.AddNodeToModifiedQueue(g, node.NodeKey)
			}
		}
		// 在此处可以调用规则的 reduce 方法
		if reduction.Reduce(g, removedNodes, solutoinNodes) {
			progress = true
			break // 如果图有变化，跳出循环并重新检查
		}
	}
	//// 规则3：低权重排除
	//g.lowWeightExclusion(solutoinNodes)
	////println("约简效率1", float64(len(removedNodes))/float64(len(g.Nodes)))
	//// 规则2：孤立节点处理
	////g.isolatedNodesHandling(removedNodes)
	//// 规则1：支配节点削减
	//g.dominanceReduction(removedNodes)

	//println("约简效率2", float64(len(removedNodes))/float64(len(g.Nodes)))
	//println("约简效率3", float64(len(removedNodes))/float64(len(g.Nodes)))
	//g.alternativeReduction(removedNodes)
	//println("约简效率", float64(len(removedNodes))/float64(len(g.Nodes)))
	return removedNodes, solutoinNodes, progress
}

// 处理孤立节点
func (g *Graph) isolatedNodesHandling(removedNodes map[string]struct{}) {
	for nodeKey := range g.Nodes {
		if _, removed := removedNodes[nodeKey]; removed {
			continue
		}

		neighbors := g.getNeighbors(nodeKey)
		if len(neighbors) == 0 {
			// 孤立节点可以直接加入解集
			// 在预处理中我们只标记它们，不直接加入解集
			removedNodes[nodeKey] = struct{}{}
		}
	}
}

// 计算 map[string]bool 的哈希值
func (s *Solution) hash() string {
	// 将 map 转化为一个排序后的字符串表示
	var keys []string
	for key := range s.solution {
		keys = append(keys, key)
	}

	// 排序确保相同的键顺序可以得到相同的哈希值
	sort.Strings(keys)

	var sb strings.Builder
	for _, key := range keys {
		// 对每个 key 和对应的 bool 值，拼接字符串 "key:true" 或 "key:false"
		sb.WriteString(key)
		sb.WriteString(":")
		sb.WriteString(strconv.FormatBool(s.solution[key]))
		sb.WriteString(",") // 用逗号分隔
	}

	// 获取最终的字符串表示
	str := sb.String()

	// 计算哈希值
	//hash := sha256.New()
	//hash.Write([]byte(str))
	//return hex.EncodeToString(hash.Sum(nil))
	return str
}

// 邻域约简
func (g *Graph) neighborhoodReduction(removedNodes map[string]struct{}) {
	for nodeKey := range g.Nodes {
		if _, removed := removedNodes[nodeKey]; removed {
			continue
		}

		neighbors := g.getNeighbors(nodeKey)
		for _, neighbor := range neighbors {
			neighborNeighbors := g.getNeighbors(neighbor)
			neighborNeighbors = removeElement(neighborNeighbors, nodeKey, func(s string, s2 string) bool {
				return s == s2
			})
			if g.isSubset(neighborNeighbors, neighbors) {

				// 如果邻居的邻居完全包含在当前节点的邻居集内，删除当前节点
				if g.Nodes[nodeKey].Weight < g.Nodes[neighbor].Weight {
					removedNodes[nodeKey] = struct{}{}
				} else {
					removedNodes[neighbor] = struct{}{}
				}
			}
		}
	}
}

// 判断一个节点的邻居集是否包含另一个邻居集
func (g *Graph) isSubset(subset, set []string) bool {
	setMap := make(map[string]struct{})
	for _, node := range set {
		setMap[node] = struct{}{}
	}
	for _, node := range subset {
		if _, found := setMap[node]; !found {
			return false
		}
	}
	return true
}

// 判断两个集合是否相等
func equalSets(set1, set2 map[string]bool) bool {
	if len(set1) != len(set2) {
		return false
	}
	for key := range set1 {
		if !set2[key] {
			return false
		}
	}
	return true
}

// 移除指定节点后返回新的邻居集合
func removeNodeFromSet(neighbors map[string]bool, nodeKey string) map[string]bool {
	newNeighbors := make(map[string]bool)
	for key := range neighbors {
		if key != nodeKey {
			newNeighbors[key] = true
		}
	}
	return newNeighbors
} // Alternative约简规则
func (g *Graph) alternativeReduction(removedNodes map[string]struct{}) {
	for nodeKeyV := range g.Nodes {
		if _, removed := removedNodes[nodeKeyV]; removed {
			continue
		}

		// 获取节点v的邻居集合
		//neighborsV := g.getNeighbors(nodeKeyV)

		// 遍历所有邻居节点集合S
		for nodeKeyU := range g.Nodes {
			_, removed := removedNodes[nodeKeyU]

			if nodeKeyV == nodeKeyU || removed {
				continue
			}

			// 获取节点u的邻居集
			neighborsU := g.getNeighbors(nodeKeyU)

			// 获取邻居集S
			S := make(map[string]bool)
			S[nodeKeyU] = true
			for _, neighbor := range neighborsU {
				S[neighbor] = true
			}

			// 检查是否满足条件：
			// 1. S中的任意两点不相邻
			// 2. Σ w(u) > w(v)
			// 3. N(S) ⊆ N[v]
			if g.isValidAlternativeSet(S, nodeKeyV) && g.sumWeight(S) > g.Nodes[nodeKeyV].Weight {
				// 删除节点v
				removedNodes[nodeKeyV] = struct{}{}
				break
			}
		}
	}
}

// 判断集合S是否有效（即S中的节点不相邻，且N(S) ⊆ N[v]）

func (g *Graph) isValidAlternativeSet(S map[string]bool, nodeKeyV string) bool {
	// 检查S中的任意两个节点是否相邻
	for nodeKey1 := range S {
		for nodeKey2 := range S {
			if nodeKey1 != nodeKey2 && g.Edges[nodeKey1][nodeKey2] {
				return false
			}
		}
	}

	// 检查N(S) ⊆ N[v]
	//neighborsV := g.getNeighbors(nodeKeyV)
	for nodeKey := range S {
		neighborsS := g.getNeighbors(nodeKey)
		for _, neighbor := range neighborsS {
			if !g.Edges[nodeKeyV][neighbor] {
				return false
			}
		}
	}

	return true
}

// // 计算权重和
func (g *Graph) sumWeight(S map[string]bool) float64 {
	sum := 0.0
	for nodeKey := range S {
		sum += g.Nodes[nodeKey].Weight
	}
	return sum
}

// findMateAnd2TightNeighbor 根据tightness和mu值找到一个伴随节点w和2-tight邻居x
func (s *Solution) findMateAnd2TightNeighbor(lastNode string, path []string) (string, string) {
	var bestW, bestX string
	bestGain := -1.0 // 初始化一个很低的增益值，用来寻找最大增益的伴随节点和邻居

	// 遍历lastNode的邻居，寻找符合条件的2-tight邻居
	for neighbor := range s.g.Edges[lastNode] {
		// 排除已经在路径中的节点，避免路径上重复节点
		if contains(path, neighbor) {
			continue
		}

		// 如果邻居的tightness值 >= 1，表示它是2-tight邻居
		if s.tightness[neighbor] >= 2 {
			// 遍历2-tight邻居的邻居，寻找最佳伴随节点w
			for secondNeighbor := range s.g.Edges[neighbor] {
				// 排除已经在路径中的节点
				if contains(path, secondNeighbor) || secondNeighbor == lastNode {
					continue
				}

				// 计算该节点对的增益，依据mu值选择最大增益
				gain := s.mu[secondNeighbor] - s.mu[neighbor]
				if gain > bestGain {
					bestGain = gain
					bestW = secondNeighbor // 选择伴随节点w
					bestX = neighbor       // 选择2-tight邻居x
				}
			}
		}
	}

	return bestX, bestW
}

// AAPMoves 尝试执行交替增广路径的移动
func (s *Solution) AAPMoves() bool {
	// 遍历图中的所有节点
	for v := range s.g.Nodes {
		// 仅考虑增益大于零的节点
		if s.mu[v] > 0 && !s.solution[v] {
			// 遍历v的邻居，寻找1-tight邻居
			for neighbor := range s.solution {
				if s.g.isNeighbor(v, neighbor) {
					// 如果邻居的tightness为0，表示它是1-tight邻居
					if s.tightness[neighbor] == 1 {
						// 初始化路径
						u := neighbor
						path := []string{v, u}
						uIsInS := true // 标记u是否在S中

						// 尝试扩展路径
						for {
							lastNode := path[len(path)-1]
							var w, x string

							// 根据uIsInS判断当前扩展的节点类型
							if uIsInS {
								// 找到伴随节点w和2-tight邻居x来扩展路径
								x, w = s.findMateAnd2TightNeighbor(lastNode, path)
							} else {
								// 找到下一个2-tight邻居节点w
								w, x = s.findMateAnd2TightNeighbor(lastNode, path)
							}

							// 如果没有找到有效的节点对，停止扩展路径
							if w == "" || x == "" {
								break
							}

							// 将节点添加到路径中
							path = append(path, w, x)
							uIsInS = !uIsInS // 交替更新uIsInS
						}

						// 如果得到了有效路径，尝试翻转它
						if len(path) > 2 {
							gain := s.calculateGainForPath(path)
							if gain > 0 {
								// 翻转路径并更新集合S
								s.flipPath(path)
								return true // 移动成功
							}
						}
					}
				}
			}
		}
	}
	return false // 未找到改进的移动
}

// calculateGainForPath 计算翻转路径后的增益
func (s *Solution) calculateGainForPath(path []string) float64 {
	gain := 0.0
	for i, node := range path {
		if i%2 == 0 { // 奇数位置的节点属于独立集，需要移除
			gain -= s.mu[node]
		} else { // 偶数位置的节点不在独立集中，需要加入
			gain += s.mu[node]
		}
	}
	return gain
}

// flipPath 翻转路径，将路径上的节点从独立集中移除，未在独立集中的节点加入独立集
func (s *Solution) flipPath(path []string) {
	for i, node := range path {
		if i%2 == 0 { // 奇数位置的节点属于独立集，需要移除
			s.removeVertex(node)
		} else { // 偶数位置的节点不在独立集中，需要加入
			s.addVertex(node)
		}
	}
}

// contains 判断路径中是否包含指定的节点
func contains(path []string, node string) bool {
	for _, p := range path {
		if p == node {
			return true
		}
	}
	return false
}

func (g *Graph) UpperBound(nodeKey string, queue *NodeQueue) float64 {
	// Gather remaining nodes
	//includedNodeWeight := g.Nodes[nodeKey].Weight
	nodes := make([]*GraphNode, 0)
	for _, node := range g.Nodes {
		if nodeKey == "" {
			nodes = append(nodes, node)
		} else {

			if !queue.Contain(node.NodeKey) {
				nodes = append(nodes, node)
			}
		}
	}

	// Sort by descending weight, break ties by degree
	sort.Slice(nodes, func(i, j int) bool {
		if nodes[i].Weight != nodes[j].Weight {
			return nodes[i].Weight > nodes[j].Weight
		}
		return len(g.getNeighbors(nodes[i].NodeKey)) > len(g.getNeighbors(nodes[j].NodeKey))
	})

	// Compute node mapping
	nodeMapping := make(map[string]int)
	for i, node := range nodes {
		nodeMapping[node.NodeKey] = i
	}

	// Initialize cliques
	n := len(nodes)
	clique := make([]int, n)
	cliqueWeight := make([]float64, n)
	cliqueSizes := make([]int, n)
	covered := make([]bool, n)

	for i := 0; i < n; i++ {
		clique[i] = i
		cliqueWeight[i] = 0
		cliqueSizes[i] = 0
		covered[i] = false
	}

	neighCliqueSizes := make([]int, n)

	// Find heaviest neighboring clique for each node
	for _, node := range nodes {
		v := nodeMapping[node.NodeKey]
		heaviestClique := v

		// Reset neighbor clique sizes
		for i := 0; i < n; i++ {
			neighCliqueSizes[i] = 0
		}

		for _, neighbor := range g.getNeighbors(node.NodeKey) {
			if _, ok := g.Nodes[neighbor]; ok {
				w := nodeMapping[neighbor]

				if covered[w] {
					c := clique[w]
					neighCliqueSizes[c]++

					if neighCliqueSizes[c] == cliqueSizes[c] && cliqueWeight[c] > cliqueWeight[heaviestClique] {
						heaviestClique = c
					}
				}
			}
		}

		// Update clique weights and sizes
		clique[v] = heaviestClique
		cliqueWeight[heaviestClique] = math.Max(cliqueWeight[heaviestClique], node.Weight)
		cliqueSizes[heaviestClique]++

		// Mark node as covered
		covered[v] = true
	}

	// Keep unique cliques
	uniqueCliques := make(map[int]bool)
	for i := 0; i < n; i++ {
		uniqueCliques[clique[i]] = true
	}

	// Add clique weights to get upper bound
	upperBound := 0.0
	for c := range uniqueCliques {
		upperBound += cliqueWeight[c]
	}

	return upperBound
}

func (g *Graph) CanUpper(s *Solution, queue *NodeQueue) (bool, float64) {
	nodes := make([]*GraphNode, 0)
	isFlag := make(map[string]bool, 0)
	answer := ""
	for _, node := range *queue {
		if s.solution[node.NodeKey] {
			answer = node.NodeKey
		}
		nodes = append(nodes, g.Nodes[node.NodeKey])
	}

	// Sort by descending weight, break ties by degree
	sort.Slice(nodes, func(i, j int) bool {
		if nodes[i].Weight != nodes[j].Weight {
			return nodes[i].Weight > nodes[j].Weight
		}
		return nodes[i].NodeKey > nodes[j].NodeKey
	})

	for key1, _ := range s.GetSolution() {
		node := g.Nodes[key1]

		switch node.NodeType {
		case DroneToTask:
			{
				for _, node1 := range nodes {
					if node1.TaskPoint != nil && node1.TaskPoint.Id == node.TaskPoint.Id {
						isFlag[node1.NodeKey] = true
					}
				}
			}
		case DroneWorkerToTask:
			{
				for _, node1 := range nodes {
					if node1.TaskPoint != nil && node1.TaskPoint.Id == node.TaskPoint.Id || node1.Worker != nil && node1.Worker.Id == node.Worker.Id {
						isFlag[node1.NodeKey] = true
					}
				}
			}
		case WorkerToTask:
			{
				for _, node1 := range nodes {
					if node1.NodeType != WorkerToTask {
						if node1.TaskPoint != nil && node1.TaskPoint.Id == node.TaskPoint.Id || node1.Worker != nil && node1.Worker.Id == node.Worker.Id {
							isFlag[node1.NodeKey] = true
						}
					}

				}
			}
		case DroneCarToChargePoint:
			{
				for _, node1 := range nodes {
					if node1.Car != nil && node1.Car.Id == node.Car.Id || node1.ChargePoint != nil && node1.ChargePoint.Id == node.ChargePoint.Id {
						isFlag[node1.NodeKey] = true
					}
				}
			}
		case CarToChargePoint:
			{
				for _, node1 := range nodes {
					if node1.NodeType != CarToChargePoint {
						if node1.ChargePoint != nil && node1.ChargePoint.Id == node.ChargePoint.Id || node1.Car != nil && node1.Car.Id == node.Car.Id {
							isFlag[node1.NodeKey] = true
						}
					}
				}
			}
		case DroneToChargePoint:
			{
				for _, node1 := range nodes {
					if node1.ChargePoint != nil && node1.ChargePoint.Id == node.ChargePoint.Id {
						isFlag[node1.NodeKey] = true
					}
				}
			}
		default:
			continue
		}
	}
	if answer == "" {
		if queue.Len() > 0 && nodes[0].Weight == 0 {
			return false, 0
		}
		return true, nodes[0].Weight
	}
	for _, node := range nodes {
		if node.Weight > g.Nodes[answer].Weight { //在他之上
			if !isFlag[node.NodeKey] {
				return true, node.Weight - g.Nodes[answer].Weight
			}
		}

	}
	return false, 0
}
