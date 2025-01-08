package core

import (
	"math"
	"sort"
)

// Reduction 定义了所有约简规则应该实现的接口
type Reduction interface {
	GetReductionType() string
	Reduce(g *Graph, removeNodes map[string]struct{}, solutionNodes map[string]struct{}) bool // 应用该约简规则并返回是否有任何变化
	ClearModifiedQueue()
	AddNodeToModifiedQueue(g *Graph, nodeKey string)
	Init()
}

// 支配约简规则
type DominanceReduction struct {
	ModifiedQueue []string
}

func (r *DominanceReduction) GetReductionType() string {
	return "DominanceReduction"
}
func (r *DominanceReduction) Init() {
	r.ModifiedQueue = make([]string, 0)
}
func (r *DominanceReduction) Reduce(g *Graph, removeNodes map[string]struct{}, solutionNodes map[string]struct{}) bool {
	// 实现邻域约简逻辑
	progress := false
	queue := r.ModifiedQueue
	// 清空队列，因为已经处理过这些节点
	r.ClearModifiedQueue()

	for _, nodeKey := range queue {
		if _, removed := removeNodes[nodeKey]; removed || g.Nodes[nodeKey] == nil {
			continue
		}
		//neighbors := g.getNeighbors(nodeKey)
		for _, neighbor := range queue {
			if _, removed := removeNodes[neighbor]; removed || g.Nodes[neighbor] == nil {
				continue
			}
			if nodeKey != neighbor {
				if r.isDominated(g, nodeKey, neighbor) {
					removeNodes[neighbor] = struct{}{}
					r.AddNodeToModifiedQueue(g, neighbor) // 加入修改队列
					progress = true
					break
				}
			}

		}

	}

	return progress
}

// Graph 方法: 将节点加入修改队列
func (r *DominanceReduction) ClearModifiedQueue() {
	r.ModifiedQueue = make([]string, 0)

}

// Graph 方法: 将节点加入修改队列
func (r *DominanceReduction) AddNodeToModifiedQueue(g *Graph, nodeKey string) {
	// 确保节点没有重复添加
	for _, key := range r.ModifiedQueue {
		if key == nodeKey {
			return
		}
	}
	r.ModifiedQueue = append(r.ModifiedQueue, nodeKey)
	for _, key := range g.getNeighbors(nodeKey) {
		r.ModifiedQueue = append(r.ModifiedQueue, key)
	}
}

// 判断节点u是否被节点v支配
func (r *DominanceReduction) isDominated(g *Graph, u, v string) bool {
	uNeighbors := g.getNeighbors(u)
	vNeighbors := g.getNeighbors(v)

	if g.Nodes[v].Weight > g.Nodes[u].Weight {
		return false
	}

	return g.isSubset(uNeighbors, vNeighbors)
}

// NeighborhoodReduction 定义了邻域约简规则
type NeighborhoodReduction struct {
	ModifiedQueue []string
}

func (r *NeighborhoodReduction) GetReductionType() string {
	return "NeighborhoodReduction"
}

func (r *NeighborhoodReduction) Init() {
	r.ModifiedQueue = make([]string, 0)
}

func (r *NeighborhoodReduction) Reduce(g *Graph, removeNodes map[string]struct{}, solutoinNodes map[string]struct{}) bool {
	queue := r.ModifiedQueue
	// 清空队列，因为已经处理过这些节点
	r.ClearModifiedQueue()
	progress := false
	for _, nodeKey := range queue {
		if _, removed := removeNodes[nodeKey]; removed {
			continue
		}

		neighbors := g.getNeighbors(nodeKey)
		if len(neighbors) == 0 {
			continue
		}

		// 找到邻居中的最小权重
		sumNeighborWeight := 0.0
		for _, neighbor := range neighbors {
			sumNeighborWeight += g.Nodes[neighbor].Weight
		}

		// 如果节点权重小于其所有邻居的最小权重，则可以排除
		if g.Nodes[nodeKey].Weight > sumNeighborWeight {
			solutoinNodes[nodeKey] = struct{}{}
			removeNodes[nodeKey] = struct{}{}
			for _, key := range g.getNeighbors(nodeKey) {
				removeNodes[key] = struct{}{}
			}
			progress = true
		}
	}
	return progress
}

func (r *NeighborhoodReduction) ClearModifiedQueue() {
	r.ModifiedQueue = make([]string, 0)
}

func (r *NeighborhoodReduction) AddNodeToModifiedQueue(g *Graph, nodeKey string) {
	// 确保节点没有重复添加
	for _, key := range r.ModifiedQueue {
		if key == nodeKey {
			return
		}
	}
	r.ModifiedQueue = append(r.ModifiedQueue, nodeKey)
} // Fold2Reduction 定义了度为 2 的节点约简规则
//	type Fold2Reduction struct {
//		ModifiedQueue []string
//	}
//
//	func (r *Fold2Reduction) GetReductionType() string {
//		return "Fold2Reduction"
//	}
//
//	func (r *Fold2Reduction) Init() {
//		r.ModifiedQueue = make([]string, 0)
//	}
//
//	func (r *Fold2Reduction) Reduce(g *Graph, removeNodes map[string]struct{}) bool {
//		queue := r.ModifiedQueue
//		// 清空队列，因为已经处理过这些节点
//		r.ClearModifiedQueue()
//		progress := false
//		for _, nodeKey := range queue {
//			node := g.Nodes[nodeKey]
//			if _, removed := removeNodes[nodeKey]; removed || g.Nodes[nodeKey] == nil || len(g.GetNeighbors(nodeKey)) != 2 {
//				continue
//			}
//
//			// 获取两个邻居
//			neighbors := g.GetNeighbors(nodeKey)
//			neighbor1, neighbor2 := neighbors[0], neighbors[1]
//
//			neighborWeightSum := g.Nodes[neighbor1].Weight + g.Nodes[neighbor2].Weight
//
//			// 如果节点的权重大于等于邻居权重之和，包含节点
//			if node.Weight >= neighborWeightSum {
//				g.SetNodeStatus(nodeKey, INCLUDE)
//				g.SetNodeStatus(neighbor1, INCLUDE)
//				g.SetNodeStatus(neighbor2, INCLUDE)
//				r.AddNodeToModifiedQueue(g, nodeKey)
//				progress = true
//			}
//
//			// 检查是否是三角形，并应用条件
//			if g.HasEdge(neighbor1, neighbor2) && node.Weight >= math.Max(g.Nodes[neighbor1].Weight, g.Nodes[neighbor2].Weight) {
//				g.SetNodeStatus(nodeKey, INCLUDE)
//				progress = true
//
//			}
//		}
//
//		r.ClearModifiedQueue()
//		return progress
//	}
//
//	func (r *Fold2Reduction) ClearModifiedQueue() {
//		r.ModifiedQueue = make([]string, 0)
//	}
//
//	func (r *Fold2Reduction) AddNodeToModifiedQueue(g *Graph, nodeKey string) {
//		// 确保节点没有重复添加
//		for _, key := range r.ModifiedQueue {
//			if key == nodeKey {
//				return
//			}
//		}
//		r.ModifiedQueue = append(r.ModifiedQueue, nodeKey)
//	} // CliqueReduction 定义了完全子图约简规则
//
//	type CliqueReduction struct {
//		ModifiedQueue []string
//	}
//
//	func (r *CliqueReduction) GetReductionType() string {
//		return "CliqueReduction"
//	}
//
//	func (r *CliqueReduction) Init() {
//		r.ModifiedQueue = make([]string, 0)
//	}
//
//	func (r *CliqueReduction) Reduce(g *Graph) bool {
//		progress := false
//		for _, nodeKey := range r.ModifiedQueue {
//			node := g.Nodes[nodeKey]
//
//			if node.Status == NOTSET {
//				neighbors := g.GetNeighbors(nodeKey)
//				isClique := true
//
//				// 检查是否为完全子图
//				for i := 0; i < len(neighbors); i++ {
//					for j := i + 1; j < len(neighbors); j++ {
//						if !g.HasEdge(neighbors[i], neighbors[j]) {
//							isClique = false
//							break
//						}
//					}
//					if !isClique {
//						break
//					}
//				}
//
//				// 如果是完全子图，且当前节点的权重最大，则包含该节点
//				if isClique {
//					maxWeight := node.Weight
//					for _, neighborKey := range neighbors {
//						if g.Nodes[neighborKey].Weight > maxWeight {
//							maxWeight = g.Nodes[neighborKey].Weight
//						}
//					}
//
//					if node.Weight == maxWeight {
//						g.SetNodeStatus(nodeKey, INCLUDE)
//						progress = true
//					}
//				}
//			}
//		}
//
//		r.ClearModifiedQueue()
//		return progress
//	}
//
//	func (r *CliqueReduction) ClearModifiedQueue() {
//		r.ModifiedQueue = make([]string, 0)
//	}
//
//	func (r *CliqueReduction) AddNodeToModifiedQueue(g *Graph, nodeKey string) {
//		// 确保节点没有重复添加
//		for _, key := range r.ModifiedQueue {
//			if key == nodeKey {
//				return
//			}
//		}
//		r.ModifiedQueue = append(r.ModifiedQueue, nodeKey)
//	} // CliqueNeighborhoodReductionFast 定义了快速完全子图邻域约简规则
type CliqueNeighborhoodReductionFast struct {
	ModifiedQueue []string
}

func (r *CliqueNeighborhoodReductionFast) GetReductionType() string {
	return "CliqueNeighborhoodReductionFast"
}

func (r *CliqueNeighborhoodReductionFast) Init() {
	r.ModifiedQueue = make([]string, 0)
}

func (r *CliqueNeighborhoodReductionFast) Reduce(g *Graph, removeNodes map[string]struct{}, solutoinNodes map[string]struct{}) bool {
	queue := r.ModifiedQueue
	// 清空队列，因为已经处理过这些节点
	r.ClearModifiedQueue()
	progress := false
	for _, nodeKey := range queue {
		node := g.Nodes[nodeKey]
		if _, removed := removeNodes[nodeKey]; removed {
			continue
		}
		// 获取邻居并计算权重之和
		neighborWeights := 0.0
		neighbors := g.GetNeighbors(nodeKey)
		neighborhood := make(map[string]bool)

		for _, neighborKey := range neighbors {
			neighborWeights += g.Nodes[neighborKey].Weight
			neighborhood[neighborKey] = true
		}

		if node.Weight >= neighborWeights {
			continue
		}

		// 排序邻居按权重
		sort.SliceStable(neighbors, func(i, j int) bool {
			return g.Nodes[neighbors[i]].Weight > g.Nodes[neighbors[j]].Weight
		})

		// 检查三角形并判断
		isReducible := false
		for i := 0; i < len(neighbors) && !isReducible; i++ {
			neighbor1 := neighbors[i]
			if !neighborhood[neighbor1] {
				continue
			}

			for _, neighbor2 := range g.GetNeighbors(neighbor1) {
				if neighbor2 != neighbor1 && neighborhood[neighbor2] {
					neighborWeights -= math.Min(g.Nodes[neighbor1].Weight, g.Nodes[neighbor2].Weight)
					delete(neighborhood, neighbor1)
					delete(neighborhood, neighbor2)

					if node.Weight >= neighborWeights {
						isReducible = true
					}

					break
				}
			}
		}

		if isReducible {
			solutoinNodes[nodeKey] = struct{}{}
			removeNodes[nodeKey] = struct{}{}
			for _, key := range g.getNeighbors(nodeKey) {
				removeNodes[key] = struct{}{}
			}
			progress = true
		}

	}

	r.ClearModifiedQueue()
	return progress
}

func (r *CliqueNeighborhoodReductionFast) ClearModifiedQueue() {
	r.ModifiedQueue = make([]string, 0)
}

func (r *CliqueNeighborhoodReductionFast) AddNodeToModifiedQueue(g *Graph, nodeKey string) {
	// 确保节点没有重复添加
	for _, key := range r.ModifiedQueue {
		if key == nodeKey {
			return
		}
	}
	r.ModifiedQueue = append(r.ModifiedQueue, nodeKey)
}
