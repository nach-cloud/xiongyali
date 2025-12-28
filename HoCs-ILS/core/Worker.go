package core

// Worker 结构体
type Worker struct {
	Base
	WRget       float64 //工人在每分钟的移动距离
	UpTime      float64 //上线时刻
	DownTime    float64 //下线时刻
	date        int
	UUID        string
	IsPublisher bool
	Status      AgentStatus
}

func (w *Worker) GetDate() int {
	return w.date
}
func (w *Worker) GetUpTime() float64 {
	return w.UpTime
}
func (w *Worker) GetSpeed() float64 {
	return w.WRget
}
func (w *Worker) GetDownTime() float64 {
	return w.DownTime
}
func (w *Worker) GetUUId() string {
	return w.UUID
}
func (w *Worker) SetPublisher(value bool) {
	w.IsPublisher = value
}
func (w *Worker) SetPosition(x int, y int) {
	w.X = x
	w.Y = y
}
func (w *Worker) GetIsPublisher() bool {
	return w.IsPublisher
}
func (w *Worker) GetStatus() AgentStatus {
	return w.Status
}
func (w *Worker) SetStatus(status AgentStatus) {
	w.Status = status
}
func (w *Worker) GetId() int {
	return w.Id
}

func (w *Worker) GetPosition() Position {
	return Position{X: w.X, Y: w.Y}
}

func (w *Worker) PerformTask(taskID int) {
	// 实现工作人员执行任务的逻辑
}
func (w *Worker) IsValid() bool {
	return w != nil
}
