package core

// Drone 结构体
type Drone struct {
	UUID           string
	RemainingPower float64 //现在电量移动距离
	FullPower      float64 //满电移动距离
	URget          float64 //每分钟移动距离
	date           int     //上线日期
	UpTime         float64 //上线时刻
	DownTime       float64 //下线时间
	Status         AgentStatus
	IsPublisher    bool
	NeedCharge     bool //代表当前在充电点，可以充电
	Base
}

func (d *Drone) GetType() string {
	return "Drone"
}

func (d *Drone) SetStatus(status AgentStatus) {
	d.Status = status
}
func (d *Drone) SetPosition(x int, y int) {
	d.X = x
	d.Y = y
}
func (d *Drone) GetDate() int {
	return d.date
}
func (d *Drone) GetUUId() string {
	return d.UUID
}
func (d *Drone) SetPublisher(value bool) {
	d.IsPublisher = value
}
func (d *Drone) GetUpTime() float64 {
	return d.UpTime
}
func (d *Drone) GetIsPublisher() bool {
	return d.IsPublisher
}
func (d *Drone) GetStatus() AgentStatus {
	return d.Status
}
func (d *Drone) GetId() int {
	return d.Id
}
func (d *Drone) GetSpeed() float64 {
	return d.URget
}
func (d *Drone) GetDownTime() float64 {
	return d.DownTime
}
func (d *Drone) GetPosition() Position {
	return Position{X: d.X, Y: d.Y}
}

func (d *Drone) IsValid() bool {
	return d != nil
}
