package core

type GraphNode struct {
	NodeKey     string
	index       int          //索引
	NodeType    NodeType     // 节点类型，表示这个节点的情况
	Drone       *Drone       // 无人机，可能为空
	Car         *Car         // 汽车，可能为空
	Worker      *Worker      // 工作人员，可能为空
	TaskPoint   *TaskPoint   // 任务点，可能为空
	ChargePoint *ChargePoint // 充电点，可能为空
	Weight      float64      // 节点权值
	CostPower   float64      //消耗电量
	Status      int
	Time        float64
}
type QueuesSolutions struct {
	Keys     []string
	Solution *Solution
}
type NodeQueue []GraphNode

// Len 方法返回堆中的元素数量。
func (h NodeQueue) Len() int { return len(h) }

// Less 方法报告节点i是否应该排在节点j之前。
func (h NodeQueue) Less(i, j int) bool {
	if h[i].Weight != h[j].Weight {
		return h[i].Weight > h[j].Weight
	} else {
		return h[i].NodeKey > h[j].NodeKey // 注意：这里假设我们想要一个最大堆
	}

}

// Swap 方法交换两个节点。
func (h NodeQueue) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
}

// Push 方法向堆中添加一个元素。
func (h *NodeQueue) Push(x interface{}) {

	n := len(*h)
	node := x.(GraphNode)
	node.index = n
	*h = append(*h, node)
}

// Pop 方法从堆中移除最后一个元素，并返回它。
func (h *NodeQueue) Pop() interface{} {
	n := len(*h)
	x := (*h)[n-1]  // 返回删除的元素
	*h = (*h)[:n-1] // [n:m]不包括下标为m的元素
	return x

}

func (h NodeQueue) Contain(key string) bool {
	for _, keys := range h {
		if keys.NodeKey == key {
			return true
		}
	}
	return false
}
