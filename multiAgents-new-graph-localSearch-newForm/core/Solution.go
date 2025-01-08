package core

import (
	"math/rand"
	"time"
)

// Solution 结构体表示当前的解
type Solution struct {
	g            *Graph             // 图实例
	solution     map[string]bool    // 当前解中的节点集合
	tightness    map[string]int     // 节点的 tightness 值，当前解中，有多少个邻居包含在当前解中，tightness为0说明可以直接接入独立集中
	mu           map[string]float64 // 节点的 mu 值,潜在贡献，当前节点权重-与他相邻的节点在当前解中权重之合
	weight       float64            // 当前解的总权重
	freeNodes    []string           // 自由节点列表，代表能加入当前解集合的节点
	nonFreeNodes []string           // 非自由节点列表
}

// 创建新的 Solution
func NewSolution(g *Graph) *Solution {
	s := &Solution{
		g:         g,
		solution:  make(map[string]bool),
		tightness: make(map[string]int),
		mu:        make(map[string]float64),
		weight:    0,
	}

	for nodeKey, node := range g.Nodes {
		s.mu[nodeKey] = node.Weight
		s.freeNodes = append(s.freeNodes, nodeKey)
	}

	return s
}

// 添加节点到解中
func (s *Solution) addVertex(v string) {
	s.solution[v] = true
	s.weight += s.g.Nodes[v].Weight

	// 更新 tightness 和 mu
	for _, neighbor := range s.g.getNeighbors(v) {
		s.tightness[neighbor]++
		s.mu[neighbor] -= s.g.Nodes[v].Weight

		// 如果邻居在自由节点列表中，移动到非自由节点列表
		s.moveNodeToNonFree(neighbor)
	}

	// 从自由节点列表中移除 v
	s.removeNodeFromFree(v)
}

// 从解中移除节点
func (s *Solution) removeVertex(v string) {
	delete(s.solution, v)
	s.weight -= s.g.Nodes[v].Weight

	// 更新 tightness 和 mu
	for _, neighbor := range s.g.getNeighbors(v) {
		s.tightness[neighbor]--
		s.mu[neighbor] += s.g.Nodes[v].Weight

		// 如果 tightness 为 0，将邻居移动到自由节点列表
		if s.tightness[neighbor] == 0 {
			s.moveNodeToFree(neighbor)
		}
	}

	// 将 v 添加回自由节点列表
	s.freeNodes = append(s.freeNodes, v)
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
	for v := range s.g.Nodes {
		if s.mu[v] > ZeroEpsilon && !s.solution[v] {
			// 移除 v 的所有相邻节点
			for neighbor := range s.solution {
				if s.g.isNeighbor(v, neighbor) {
					s.removeVertex(neighbor)
				}
			}
			s.addVertex(v)
			return true
		}
	}
	return false
}

// twoImprovement 方法
// 如果解集某个节点v的收益，不如把该节点放弃，选择另外两个v相邻节点收益大，那么将选择另外两个
func (s *Solution) twoImprovement() bool {
	for x := range s.solution {
		// 获取 x 的 tightness 为 1 的邻居
		oneTightNeighbors := []string{}
		for _, neighbor := range s.g.getNeighbors(x) {
			if s.tightness[neighbor] == 1 {
				oneTightNeighbors = append(oneTightNeighbors, neighbor)
			}
		}

		if len(oneTightNeighbors) < 2 {
			continue
		}

		xWeight := s.g.Nodes[x].Weight

		// 查找没有边相连的两个邻居 v 和 w
		for i := 0; i < len(oneTightNeighbors); i++ {
			v := oneTightNeighbors[i]
			for j := i + 1; j < len(oneTightNeighbors); j++ {
				w := oneTightNeighbors[j]
				if !s.g.isNeighbor(v, w) {
					vWeight := s.g.Nodes[v].Weight
					wWeight := s.g.Nodes[w].Weight

					if xWeight >= vWeight+wWeight {
						continue
					}

					s.removeVertex(x)
					s.addVertex(v)
					s.addVertex(w)
					return true
				}
			}
		}
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
