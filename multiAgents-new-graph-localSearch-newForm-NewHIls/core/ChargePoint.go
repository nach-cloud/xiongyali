package core

// 充电点 结构体
type ChargePoint struct {
	Base
	UpTime int //续航点上线时间
	Status int //0代表空闲,1代表被占用
}
