package core

type AgentStatus int
type TaskStatus int

const (
	Idle   AgentStatus = iota //发布者发布合约Idle->waiting
	Wating                    //任务完成       Busy-> Idle
	PreBusy
	Busy             //
	WaitingCharge    //代表智能体当前在等待充电中
	WaitingAssignCar //代表智能体等待分配汽车中
)
const (
	Start      TaskStatus = iota //发布者发布合约Idle->waiting
	WatingTask                   //任务完成       Busy-> Idle
	Ready
	Bound //通知
)
