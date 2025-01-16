package core

// 用于充电过程中记录充电点的信息
type ChargePointInfo struct {
	*ChargePoint
	DroneToChargeTime  float64            //到这里的无人机的总时间
	DronesToChargeTime map[string]float64 //无人机id到达充电点的时间集合
	MaxDowntime        float64            //最晚的下线时间
}

func (info *ChargePointInfo) Init(chargePoint *ChargePoint) {
	//info.DroneIds = make([]string, 0)
	info.ChargePoint = chargePoint
	info.DronesToChargeTime = make(map[string]float64)
}
