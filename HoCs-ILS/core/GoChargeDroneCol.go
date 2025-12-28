package core

// 存储联盟效能函数结果的数据结构
type GoChargeDroneCol struct {
	ChargeId  int     //充电点id
	Drone     *Drone  //无人机id
	Distance  float64 //距离
	Time      float64 //所消耗时间
	StartTime int     //开始时间

}
