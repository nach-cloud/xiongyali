package core

import "sync"

// Car 结构体
type Car struct {
	Base
	rwMu        sync.RWMutex
	UUID        string
	CRget       float64 //汽车在每分钟移动的距离
	date        int
	UpTime      float64     //上线时刻
	DownTime    float64     //下线时刻
	CarType     int         //汽车类别，0代表辅助任务汽车，1代表巡逻汽车
	Status      AgentStatus //智能体状太，默认0,  1表示waiting，2表示preBusy,3代表busy
	IsPublisher bool        //默认为0,1代表是发布者
	ChargePow   float64
}

func (c *Car) GetDate() int {
	return c.date
}
func (c *Car) SetPosition(x int, y int) {
	c.X = x
	c.Y = y
}
func (c *Car) SetPublisher(value bool) {
	c.IsPublisher = value
}
func (c *Car) GetIsPublisher() bool {
	return c.IsPublisher
}
func (c *Car) GetStatus() AgentStatus {
	c.rwMu.Lock()
	defer c.rwMu.Unlock()
	return c.Status
}
func (c *Car) SetStatus(status AgentStatus) {
	c.rwMu.Lock()
	defer c.rwMu.Unlock()
	c.Status = status
}
func (c *Car) GetUUId() string {
	return c.UUID
}
func (c *Car) GetId() int {
	return c.Id
}

func (c *Car) GetPosition() Position {
	return Position{X: c.X, Y: c.Y}
}
func (c *Car) GetUpTime() float64 {
	return c.UpTime
}

func (c *Car) GetSpeed() float64 {
	return c.CRget
}
func (c *Car) GetDownTime() float64 {
	return c.DownTime
}
func (c *Car) PerformTask(taskId int) {
	// 实现充电汽车执行任务的逻辑
}
func (c *Car) IsValid() bool {
	return c != nil
}
func (c *Car) GetType() string {
	return "Car"
}
