package core

import (
	"math"
	"math/rand"
	"time"
)

// Solution 结构体表示当前的解
type Solution struct {
	g                    *Graph                     // 图实例
	solution             map[string]bool            // 当前解中的节点集合
	tightness            map[string]int             // 节点的 tightness 值，当前解中，有多少个邻居包含在当前解中，tightness为0说明可以直接接入独立集中
	mu                   map[string]float64         // 节点的 mu 值,潜在贡献，当前节点权重-与他相邻的节点在当前解中权重之合
	S1                   map[string]bool            //1紧领域变化的集合
	S2                   map[string]bool            //2紧领域变化的集合
	SPlus                map[string]bool            //2紧领域变化的集合
	mates                map[string][2]string       //key是value的拼接
	tightNeighborhood    map[string]map[string]bool //节点1紧邻
	twoTightNeighborhood map[string]map[string]bool //节点2紧邻
	weight               float64                    // 当前解的总权重
	freeNodes            []string                   // 自由节点列表，代表能加入当前解集合的节点
	nonFreeNodes         []string                   // 非自由节点列表
}

// 创建新的 Solution
func NewSolution(g *Graph) *Solution {

	s := &Solution{
		g:                    g,
		solution:             make(map[string]bool),
		tightness:            make(map[string]int),
		mu:                   make(map[string]float64),
		freeNodes:            []string{},
		nonFreeNodes:         []string{},
		SPlus:                make(map[string]bool),
		S1:                   make(map[string]bool),
		S2:                   make(map[string]bool),
		mates:                make(map[string][2]string),
		tightNeighborhood:    make(map[string]map[string]bool),
		twoTightNeighborhood: make(map[string]map[string]bool),
	}

	for nodeKey, node := range g.Nodes {
		s.mu[nodeKey] = node.Weight
		s.freeNodes = append(s.freeNodes, nodeKey)
	}

	return s
}
func (s *Solution) addMate(u, v string) string {
	key := generateMateKey(u, v)
	if _, exists := s.mates[key]; !exists {
		s.mates[key] = [2]string{u, v}
		s.twoTightNeighborhood[key] = make(map[string]bool)
	}
	return key
}

// removeMate removes a mate pair from the mates map
func (s *Solution) removeMate(u, v string) {
	key := generateMateKey(u, v)
	delete(s.mates, key)
	delete(s.twoTightNeighborhood, key)
	delete(s.S2, key)
}

// 添加节点到解中
func (s *Solution) addVertex(v string) {

	s.solution[v] = true
	s.weight += s.g.Nodes[v].Weight

	// Update tightNeighborhood
	if _, exists := s.tightNeighborhood[v]; !exists {
		s.tightNeighborhood[v] = make(map[string]bool)
	}
	// Update tightness and mu
	for _, neighbor := range s.g.getNeighbors(v) {

		s.tightness[neighbor]++
		s.mu[neighbor] -= s.g.Nodes[v].Weight

		// Update S_plus
		if s.mu[neighbor] > ZeroEpsilon {
			s.SPlus[neighbor] = true
		} else {
			delete(s.SPlus, neighbor)
		}

		// If tightness becomes 1, add to S1
		if s.tightness[neighbor] == 1 {
			s.S1[v] = true
			s.tightNeighborhood[v][neighbor] = true
		}
		s.moveNodeToNonFree(neighbor)
	}

	// Remove from freeNodes
	s.removeNodeFromFree(v)

	//mates := make(map[string]bool)
	//for node := range s.solution {
	//	key := s.addMate(v, node)
	//	mates[key] = true
	//}

	// Update twoTightNeighborhood
	//s.updateTwoTightNeighborhoodAfterAdd(v, mates)

	// Update S2
	//s.updateS2AfterAdd(v)

}

// removeVertex removes a vertex from the solution
func (s *Solution) removeVertex(v string) {
	delete(s.solution, v)
	s.weight -= s.g.Nodes[v].Weight

	// Update tightness and mu
	for _, neighbor := range s.g.getNeighbors(v) {
		s.tightness[neighbor]--
		s.mu[neighbor] += s.g.Nodes[v].Weight

		// Update S_plus
		if s.mu[neighbor] > ZeroEpsilon {
			s.SPlus[neighbor] = true
		} else {
			delete(s.SPlus, neighbor)
		}

		// If tightness becomes 0, move to freeNodes
		if s.tightness[neighbor] == 0 {
			s.moveNodeToFree(neighbor)
		}

	}

	// Add back to freeNodes
	s.freeNodes = append(s.freeNodes, v)

	// Update tightNeighborhood
	delete(s.tightNeighborhood, v)
	//for _, neighbors := range s.tightNeighborhood {
	//	if _, exists := neighbors[v]; exists {
	//		delete(neighbors, v)
	//	}
	//}
	delete(s.S1, v)
	//for node := range s.solution {
	//	s.removeMate(v, node)
	//	s.removeMate(node, v)
	//}

	// Update twoTightNeighborhood
	//s.updateTwoTightNeighborhoodAfterRemove(v)

	// Update S2
	//s.updateS2AfterRemove(v)
}

// updateTwoTightNeighborhoodAfterAdd updates twoTightNeighborhood after adding a vertex
func (s *Solution) updateTwoTightNeighborhoodAfterAdd(m string, mates map[string]bool) {
	n := ""
	for mateKey, _ := range mates {
		u, w := s.mates[mateKey][0], s.mates[mateKey][1]
		//for _, neighbor := range s.g.getNeighbors(v) {
		//		if neighbor != u && neighbor != w {
		//			s.twoTightNeighborhood[mateKey][neighbor] = true
		//		}
		//}

		if u == m {
			n = w
		}
		if w == m {
			n = u
		}
		for neighborhood, _ := range s.tightNeighborhood[n] {
			if s.g.Edges[neighborhood][m] {
				if s.twoTightNeighborhood[mateKey] == nil {
					s.twoTightNeighborhood[mateKey] = make(map[string]bool)
				}
				s.twoTightNeighborhood[mateKey][neighborhood] = true
			}
		}
	}

}

// updateTwoTightNeighborhoodAfterRemove updates twoTightNeighborhood after removing a vertex
func (s *Solution) updateTwoTightNeighborhoodAfterRemove(v string) {
	for mateKey, pair := range s.mates {
		u, w := pair[0], pair[1]
		if v == u || v == w {
			// Recalculate twoTightNeighborhood for this mate
			s.twoTightNeighborhood[mateKey] = make(map[string]bool)
			for _, node := range []string{u, w} {
				for _, neighbor := range s.g.getNeighbors(node) {
					if !s.solution[neighbor] && neighbor != u && neighbor != w {
						s.twoTightNeighborhood[mateKey][neighbor] = true
					}
				}
			}
		}
	}
}

// updateS2AfterAdd updates S2 after adding a vertex
func (s *Solution) updateS2AfterAdd(v string) {
	for mateKey, pair := range s.mates {
		u, w := pair[0], pair[1]
		if v == u || v == w {
			s.S2[mateKey] = true
		}
	}
}

// updateS2AfterRemove updates S2 after removing a vertex
func (s *Solution) updateS2AfterRemove(v string) {
	for mateKey, pair := range s.mates {
		u, w := pair[0], pair[1]
		if v == u || v == w {
			s.S2[mateKey] = true
		}
	}
}

// 添加随机节点
func (s *Solution) addRandomVertex() {
	if len(s.freeNodes) == 0 {
		return
	}
	rand.Seed(time.Now().UnixNano())
	idx := rand.Intn(len(s.freeNodes))
	v := s.freeNodes[idx]
	s.addVertex(v)
}

// 检查解的完整性
func (s *Solution) integrityCheck() bool {
	for v := range s.solution {
		for _, neighbor := range s.g.getNeighbors(v) {
			if s.solution[neighbor] {
				return false
			}
		}
	}
	return true
}

// 检查是否是最大解
func (s *Solution) isMaximal() bool {
	for _, v := range s.freeNodes {
		canAdd := true
		for neighbor := range s.solution {
			if s.g.isNeighbor(v, neighbor) {
				canAdd = false
				break
			}
		}
		if canAdd {
			return false
		}
	}
	return true
}

// omegaImprovement 方法
// 在空闲节点中，找到能加入进来产生增益的节点加入
func (s *Solution) omegaImprovement() bool {
	for u := range s.SPlus {
		if s.solution[u] {
			continue
		}
		// Apply (∗,1) move: insert u and remove its neighbors
		for _, neighbor := range s.g.getNeighbors(u) {
			if s.solution[neighbor] {
				s.removeVertex(neighbor)
			}
		}
		s.addVertex(u)
		return true // After applying a move, return to allow iteration
	}
	return false
}

// oneStarMoves applies (1,∗) moves
func (s *Solution) oneStarMoves() bool {
	s1 := s.S1
	s2 := make(map[string]bool)
	for key, value := range s1 {
		s2[key] = value
	}
	for u := range s2 {
		// Remove u

		// Find independent set I to add
		I := s.findIndependentSetAfterRemoval(u)

		if len(I) == 0 {
			delete(s.S1, u)
			continue
		}
		s.removeVertex(u)
		// Add independent set I
		for _, node := range I {
			s.addVertex(node)
		}
		delete(s.S1, u)
		return true
	}
	return false
}

// findIndependentSetAfterRemoval finds an independent set after removing a node
func (s *Solution) findIndependentSetAfterRemoval(u string) []string {
	I := []string{}
	values := s.g.Edges[u]
	neighbors := make(map[string]bool)
	for key, value := range values {
		neighbors[key] = value
	}

	// Recursive or greedy algorithm can be implemented here
	// For simplicity, using a greedy approach
	for node := range neighbors {
		if s.tightness[node] == 1 && s.mu[node] > ZeroEpsilon && !s.solution[node] && neighbors[node] {
			I = append(I, node)
			// Remove node's neighbors from consideration
			for neighbor := range s.g.Edges[node] {
				neighbors[neighbor] = false
			}
		}
	}
	return I
}

// findIndependentSetAfterTwoRemoval finds an independent set after removing two nodes
func (s *Solution) findIndependentSetAfterTwoRemoval(u, v string) []string {
	I := []string{}
	neighbors := make(map[string]bool)
	for node := range s.g.Edges[u] {
		neighbors[node] = true
	}
	for node := range s.g.Edges[v] {
		neighbors[node] = true
	}

	// Randomly shuffle the nodes for random addition
	nodes := []string{}
	for node := range neighbors {
		if s.tightness[node] == 2 && s.mu[node] > ZeroEpsilon && !s.solution[node] {
			nodes = append(nodes, node)
		}
	}
	rand.Shuffle(len(nodes), func(i, j int) {
		nodes[i], nodes[j] = nodes[j], nodes[i]
	})

	for _, node := range nodes {
		if neighbors[node] {
			I = append(I, node)
			// Remove node's neighbors from consideration
			for neighbor := range s.g.Edges[node] {
				neighbors[neighbor] = false
			}
		}

	}
	return I
}

// twoStarMoves applies (2,∗) moves
func (s *Solution) twoStarMoves() bool {
	s1 := s.S2
	s2 := make(map[string]bool)
	for key, value := range s1 {
		s2[key] = value
	}
	for mateKey := range s2 {
		u, v := s.mates[mateKey][0], s.mates[mateKey][1]

		// Find independent set I to add
		I := s.findIndependentSetAfterTwoRemoval(u, v)
		if len(I) == 0 {
			delete(s.S2, mateKey)
			continue
		}
		// Remove u and v
		s.removeVertex(u)
		s.removeVertex(v)
		// Randomly add nodes to I
		for _, node := range I {
			s.addVertex(node)
		}

		return true // Apply one move at a time
	}
	return false
}

// force 方法
func (s *Solution) force(k int) {
	rand.Seed(time.Now().UnixNano())
	for i := 0; i < k; i++ {
		if len(s.nonFreeNodes) == 0 {
			break
		}
		idx := rand.Intn(len(s.nonFreeNodes))
		v := s.nonFreeNodes[idx]

		// 移除 v 的所有相邻节点
		for neighbor := range s.solution {
			if s.g.isNeighbor(v, neighbor) {
				s.removeVertex(neighbor)
			}
		}
		s.addVertex(v)
	}
}

// 辅助方法
func (s *Solution) moveNodeToNonFree(v string) {
	s.removeNodeFromFree(v)
	if !s.containsNode(s.nonFreeNodes, v) {
		s.nonFreeNodes = append(s.nonFreeNodes, v)
	}
}

func (s *Solution) moveNodeToFree(v string) {
	s.removeNodeFromNonFree(v)
	if !s.containsNode(s.freeNodes, v) {
		s.freeNodes = append(s.freeNodes, v)
	}
}

func (s *Solution) removeNodeFromFree(v string) {
	s.freeNodes = s.removeNode(s.freeNodes, v)
}

func (s *Solution) removeNodeFromNonFree(v string) {
	s.nonFreeNodes = s.removeNode(s.nonFreeNodes, v)
}

func (s *Solution) removeNode(slice []string, v string) []string {
	for i, node := range slice {
		if node == v {
			return append(slice[:i], slice[i+1:]...)
		}
	}
	return slice
}

func (s *Solution) containsNode(slice []string, v string) bool {
	for _, node := range slice {
		if node == v {
			return true
		}
	}
	return false
}

func (s *Solution) GetSolution() map[string]bool {
	return s.solution
}

// 改进的扰动方法
func (s *Solution) improvedForce(iter int, k int) {
	rand.Seed(time.Now().UnixNano())
	for i := 0; i < k; i++ {
		if len(s.nonFreeNodes) == 0 {
			break
		}
		idx := rand.Intn(len(s.nonFreeNodes))
		v := s.nonFreeNodes[idx]

		// 移除 v 的所有相邻节点
		for neighbor := range s.solution {
			if s.g.isNeighbor(v, neighbor) {
				s.removeVertex(neighbor)
			}
		}
		s.addVertex(v)
	}
}

// 添加高频率节点
func (s *Solution) addHighFrequencyVertex(fa *FrequencyAnalyzer) {
	var bestNode string
	bestFreq := -1.0

	for _, node := range s.freeNodes {
		if fa.nodeFreq[node] > bestFreq {
			canAdd := true
			for neighbor := range s.solution {
				if s.g.isNeighbor(node, neighbor) {
					canAdd = false
					break
				}
			}
			if canAdd {
				bestNode = node
				bestFreq = fa.nodeFreq[node]
			}
		}
	}

	if bestNode != "" {
		s.addVertex(bestNode)
	}
}

// 更新频率信息
func (fa *FrequencyAnalyzer) updateFrequencies(solution *Solution) {
	// 将当前解加入迭代历史
	currentSol := make([]string, 0)
	for node := range solution.solution {
		currentSol = append(currentSol, node)
	}

	// 维护滑动窗口
	fa.iterations = append(fa.iterations, currentSol)
	if len(fa.iterations) > fa.windowSize {
		fa.iterations = fa.iterations[1:]
	}

	// 重新计算频率
	fa.recalculateFrequencies()
}

// 初始化频率分析器
func NewFrequencyAnalyzer(g *Graph, windowSize int, decayFactor float64) *FrequencyAnalyzer {
	c := &FrequencyAnalyzer{
		nodeFreq:    make(map[string]float64),
		edgeFreq:    make(map[string]map[string]float64),
		windowSize:  windowSize,
		decayFactor: decayFactor,
		iterations:  make([][]string, 0),
	}
	min := 0.01
	max := 0.1

	for node := range g.Nodes {
		randomNumber := min + rand.Float64()*(max-min)
		c.nodeFreq[node] = randomNumber
	}
	for node1, node2 := range g.Edges {
		randomNumber := min + rand.Float64()*(max-min)
		for key := range node2 {
			if c.edgeFreq[node1] == nil {
				c.edgeFreq[node1] = map[string]float64{}
			}
			c.edgeFreq[node1][key] = randomNumber
		}
	}
	return c
}

//// 局部扰动
//func (s *Solution) localPerturbation(k int, fa *FrequencyAnalyzer) {
//	candidates := s.selectLowFrequencyNodes(fa, k)
//
//	for _, node := range candidates {
//		if rand.Float64() < 0.3 { // 30%概率进行扰动
//			s.removeVertex(node)
//			// 添加高频率邻居节点
//			neighbors := s.g.getNeighbors(node)
//			for _, neighbor := range neighbors {
//				if fa.nodeFreq[neighbor] > fa.getAverageFrequency() {
//					s.addVertex(neighbor)
//					break
//				}
//			}
//		}
//	}
//}
//
//// 全局扰动
//func (s *Solution) globalPerturbation(k int, fa *FrequencyAnalyzer) {
//	removedCount := 0
//	for node := range s.solution {
//		if removedCount >= k {
//			break
//		}
//		if fa.nodeFreq[node] < fa.getAverageFrequency() {
//			s.removeVertex(node)
//			removedCount++
//		}
//	}
//
//	// 重建解
//	for !s.isMaximal() {
//		s.addHighFrequencyVertex(fa)
//	}
//}

// 重新计算频率
func (fa *FrequencyAnalyzer) recalculateFrequencies() {
	//fa.nodeFreq = make(map[string]float64)
	//fa.edgeFreq = make(map[string]map[string]float64)

	for i, solution := range fa.iterations {
		weight := math.Pow(fa.decayFactor, float64(len(fa.iterations)-i-1))

		// 更新节点频率
		for _, node := range solution {
			fa.nodeFreq[node] += weight
		}

		// 更新边频率
		for i := 0; i < len(solution); i++ {
			for j := i + 1; j < len(solution); j++ {
				if fa.edgeFreq[solution[i]] == nil {
					fa.edgeFreq[solution[i]] = make(map[string]float64)
				}
				fa.edgeFreq[solution[i]][solution[j]] += weight
			}
		}
	}
}

// 频率分析器中获取平均频率
func (fa *FrequencyAnalyzer) getAverageFrequency(solution *Solution) float64 {
	if len(solution.solution) == 0 {
		return 0
	}
	sum := 0.0
	for key := range solution.solution {
		sum += fa.nodeFreq[key]
	}
	return sum / float64(len(solution.solution))
}

// 选择低频率节点
func (s *Solution) selectLowFrequencyNodes(fa *FrequencyAnalyzer, k int) []string {
	candidates := make([]string, 0)
	avgFreq := fa.getAverageFrequency(s)

	for node := range s.solution {
		if fa.nodeFreq[node] < avgFreq {
			candidates = append(candidates, node)
		}
	}

	// 如果候选节点不足k个，返回所有候选节点
	if len(candidates) <= k {
		return candidates
	}

	// 随机选择k个节点
	rand.Shuffle(len(candidates), func(i, j int) {
		candidates[i], candidates[j] = candidates[j], candidates[i]
	})
	return candidates[:k]
}

// 选择高频率节点
func (s *Solution) selectHighFrequencyNodes(fa *FrequencyAnalyzer, k int) []string {
	candidates := make([]string, 0)
	avgFreq := fa.getAverageFrequency(s)

	for node := range s.solution {
		if fa.nodeFreq[node] > avgFreq {
			candidates = append(candidates, node)
		}
	}

	// 如果候选节点不足k个，返回所有候选节点
	if len(candidates) <= k {
		// candidates := make([]string, 0)
		// // 随机选择k个节点
		// // 获取所有键
		// keys := make([]string, 0, len(s.solution))
		// for key := range s.solution {
		// 	keys = append(keys, key)
		// }

		// // 随机打乱键的顺序
		// rand.Shuffle(len(keys), func(i, j int) {
		// 	keys[i], keys[j] = keys[j], keys[i]
		// }	)
		// for _,key := range keys[:k-len(candidates)] {
		// 	candidates = append(candidates, key)
		// }
		return candidates
	}

	// 随机选择k个节点
	rand.Shuffle(len(candidates), func(i, j int) {
		candidates[i], candidates[j] = candidates[j], candidates[i]
	})
	return candidates[:k]
}

// 选择随机节点
func (s *Solution) selectRandomNodes(fa *FrequencyAnalyzer, k int) []string {
	candidates := make([]string, 0)

	// candidates := make([]string, 0)
	// // 随机选择k个节点
	// // 获取所有键
	keys := make([]string, 0, len(s.solution))
	for key := range s.solution {
		keys = append(keys, key)
	}

	// 随机打乱键的顺序
	rand.Shuffle(len(keys), func(i, j int) {
		keys[i], keys[j] = keys[j], keys[i]
	})
	min := k
	if len(keys) < k {
		min = len(keys)
	}
	for _, key := range keys[:min] {
		candidates = append(candidates, key)
	}
	return candidates
}
