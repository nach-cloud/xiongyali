package core

// Agent 接口定义智能体的基本行为
type Agent interface {
	GetUUId() string
	GetId() int
	GetDate() int
	GetPosition() Position
	SetPosition(int, int)
	GetStatus() AgentStatus
	GetUpTime() float64
	GetDownTime() float64
	GetSpeed() float64 //一个决策时间能跑多远
	SetStatus(status AgentStatus)
	IsValid() bool
	GetType() string
	SetDownTime(float65 float64)
}
