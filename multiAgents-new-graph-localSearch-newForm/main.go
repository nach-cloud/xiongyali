package main

import (
	"encoding/csv"
	"fmt"
	"multiAgents/core"
	"os"
	"sort"
	"time"
)

var agents []core.Agent                //当前在线的智能体
var taskPoints map[int]*core.TaskPoint //对于taskPoint结构体的修改都只是副本修改！！！！
var chargePoints map[int]*core.ChargePoint
var uuidAgent map[string]core.Agent //实时的空闲智能体数目 //本次决策前的空闲智能体数目（每次决策中会变）

func main() {

	name := "data/Random_data1/testRealData10min.csv"
	folers := core.DataProcess(name)
	f, err := os.Open(name)
	defer f.Close()
	if err != nil {
		panic(err)
	}
	reader := csv.NewReader(f)
	records, _ := reader.ReadAll()
	elapsed := 0.0
	for _, folder := range folers {
		core.DecideTime = folder.DecideTime
		core.DecideCount = folder.DecideCount
		if core.QuChong(records, folder) {
			continue
		}
		var err error
		var allTakPoints map[int]*core.TaskPoint      //系统所有任务点
		var allChargePoints map[int]*core.ChargePoint //系统所有充电点
		var allAgents []core.Agent                    //系统所有智能体
		resultArr := make([]float64, 10)
		results := 0.0
		FolerResults := make([]float64, 0)
		println(folder.Name, "文件已经开始,当前decideTime", core.DecideTime, "decideCount", core.DecideCount)
		for i := 1; i <= 10; i++ {
			start := time.Now()
			println("第", i, "轮计算")
			taskPoints = make(map[int]*core.TaskPoint)
			chargePoints = make(map[int]*core.ChargePoint)
			allAgents = make([]core.Agent, 0)
			actions := make([]core.Action, 0)
			completeTaskCount := new(int)
			var dates []bool
			//allDistance = 0.0
			//pre := "data/random/20_20-80-30-90-30_13_20"

			pre := "./../1.6data/1.6data1/" + folder.Name
			_, allAgents, allTakPoints, allChargePoints, uuidAgent, dates, err = core.LoadMapData(pre+"/uavMap.xlsx", pre+"/workerMap.xlsx", pre+"/carMap.xlsx", pre+"/taskMap.xlsx", pre+"/chargeMap.xlsx")

			if err != nil {
				fmt.Println("Error loading map data:", err)
				return
			}

			//将任务点拷贝一份副本
			for key, value := range allTakPoints {
				taskPoints[key] = value
			}

			chargePoints = allChargePoints
			count := 0.0

			for index, date := range dates {
				if date {
					//println("第", index, "天")
					count++
					for i := 1; i <= core.DecideCount; i++ { //i代表第几次决策
						fmt.Println("以下第", i, "轮决策：    ")
						actions = updateAgent(allAgents, i, index, actions) //更新无人机坐标和电量等信息
						chargeTask()                                        //更新任务点最近充电点
						//无人机集合
						drones := make([]*core.Drone, 0)
						//工作人员切片
						workers := make([]*core.Worker, 0)
						workerAvgSpeed := 0.0
						cars := make([]*core.Car, 0)
						for _, agent := range agents {
							drone, ok := agent.(*core.Drone)
							worker, ok1 := agent.(*core.Worker)
							car, ok2 := agent.(*core.Car)
							if ok && drone.Status == core.Idle {
								drones = append(drones, drone)
							}
							if ok1 && worker.Status == core.Idle {
								workers = append(workers, worker)
								workerAvgSpeed = workerAvgSpeed + worker.WRget
							}
							if ok2 && car.Status == core.Idle {
								cars = append(cars, car)
							}
						}

						//构造有权无象图,各个节点的状态，各个智能体的状态
						graph := core.BuildGraph(taskPoints, drones, cars, workers, chargePoints, actions, i)

						//约简有权无向图
						s := graph.HILS()
						actions1 := execute(s, graph, i, index, taskPoints, completeTaskCount)
						actions = append(actions, actions1...)

					}
					agents = make([]core.Agent, 0)
					actions = make([]core.Action, 0)
					//更新任务点状态
					for key, value := range allTakPoints {
						taskPoints[key] = value
					}
					for _, value := range chargePoints {
						value.Status = 0
					}
				}
			}
			//println(allChargeCount)
			//println("任务完成数", *completeTaskCount)
			perResult := float64(*completeTaskCount) / (float64(len(allTakPoints)) * count)
			println("完成率为", perResult)
			results += perResult
			resultArr = append(resultArr, perResult)
			elapsed += time.Since(start).Seconds()
			println("总耗时", elapsed, "s")
		}

		println("平均值：", results/10)
		for _, result := range resultArr {
			print(result, " ")
		}
		sort.Float64s(resultArr)
		FolerResults = append(FolerResults, results/10)
		records = core.UpdateResultInThirdColumn(records, name, folder, results/10, resultArr[len(resultArr)-1], elapsed/10)
		f, _ = os.OpenFile(name, os.O_RDWR|os.O_TRUNC, 0666)
		defer f.Close()
		writer := csv.NewWriter(f)
		if err := writer.WriteAll(records); err != nil {
			println(err)
		}
	}

}

func execute(s *core.Solution, graph *core.Graph, i int, date int, points map[int]*core.TaskPoint, taskCount *int) []core.Action {
	solution := s.GetSolution()
	actions := make([]core.Action, 0)
	for key, _ := range solution {
		action := core.Action{}
		node := graph.Nodes[key]
		switch node.NodeType {
		case core.DroneToTask:
			{
				action = core.Action{Drone: node.Drone, TaskID: node.TaskPoint.Id, StartTime: i * core.DecideTime, CostPow: node.CostPower, Time: node.Time, Destination: node.TaskPoint.Position, ActionType: core.DroneToTask, Date: date}
				node.Drone.Status = core.Busy
			}
		case core.DroneWorkerToTask:
			{
				action = core.Action{Drone: node.Drone, Worker: node.Worker, TaskID: node.TaskPoint.Id, StartTime: i * core.DecideTime, CostPow: node.CostPower, Time: node.Time, Destination: node.TaskPoint.Position, ActionType: core.DroneWorkerToTask, Date: date}
				node.Drone.Status = core.Busy
				node.Worker.Status = core.Busy
				println(node.Drone.Id, "无人机与", node.Worker.Id, "工作人员结成了联盟，完成任务", node.TaskPoint.Id, "消耗时间为", node.Time, "剩余电量", node.Drone.RemainingPower, "权值", node.Weight)
				delete(points, node.TaskPoint.Id)
				(*taskCount)++
			}
		case core.WorkerToTask:
			{
				action = core.Action{Worker: node.Worker, TaskID: node.TaskPoint.Id, StartTime: i * core.DecideTime, Time: node.Time, Destination: node.TaskPoint.Position, ActionType: core.WorkerToTask, Date: date}
				node.Worker.Status = core.Busy
			}
		case core.DroneCarToChargePoint:
			{
				action = core.Action{Car: node.Car, ChargePointId: node.ChargePoint.Id, Drone: node.Drone, StartTime: i * core.DecideTime, Time: node.Time, Destination: node.ChargePoint.Position, ActionType: core.DroneCarToChargePoint, Date: date}
				node.Car.Status = core.Busy
				node.Drone.Status = core.Busy
				node.ChargePoint.Status = 1 //充电点即将有汽车赶到

				println("汽车", node.Car.Id, "在充电点", node.ChargePoint.Id, "开始去对无人机", node.Drone.Id, "的充电，消耗时间", node.Time, "剩余电量", node.Drone.RemainingPower)

			}
		case core.CarToChargePoint:
			{
				action = core.Action{Car: node.Car, StartTime: i * core.DecideTime, Time: node.Time, Destination: node.ChargePoint.Position, ActionType: core.CarToChargePoint, Date: date}
				node.Car.Status = core.Busy
			}
		case core.DroneToChargePoint:
			{
				action = core.Action{Drone: node.Drone, StartTime: i * core.DecideTime, Time: node.Time, Destination: node.ChargePoint.Position, CostPow: node.CostPower, ActionType: core.DroneToChargePoint, Date: date}
				node.Drone.Status = core.Busy
			}
		default:
			continue
		}
		actions = append(actions, action)
	}
	return actions
}

// 找到离任务点最近的充电点
func chargeTask() {
	for _, taskPoint := range taskPoints {
		if taskPoint.ChargePoint != nil {
			continue
		}
		chargeIndex, minDist := core.FoundNearChargePoisition(taskPoint.X, taskPoint.Y, chargePoints)
		taskPoint.ChargePoint = chargePoints[chargeIndex]
		taskPoint.ChargeDist = minDist
	}
}

// 更新智能体
func updateAgent(allAgents []core.Agent, i int, date int, actions []core.Action) []core.Action {
	//更新当前已完成任务智能体位置信息
	removeIndexs := make([]int, 0)
	var removeIds []string //记录下线智能体
	for index, action := range actions {
		deadLine := action.Time + float64(action.StartTime)
		//某个action截至时间到了，也就是已经完成了
		if deadLine <= float64(i*core.DecideTime) && date == action.Date {
			removeIndexs = append(removeIndexs, index)
			switch action.ActionType {
			case core.DroneToTask:
				{
					agentRelease(action.Drone, action.Destination)
					uavPowerRelease(action.Drone, action.CostPow)
				}
			case core.DroneWorkerToTask:
				{
					agentRelease(action.Drone, action.Destination)
					agentRelease(action.Worker, action.Destination)
					uavPowerRelease(action.Drone, action.CostPow)
				}
			case core.WorkerToTask:
				{
					agentRelease(action.Worker, action.Destination)
				}
			case core.DroneCarToChargePoint:
				{
					flag := true
					agentRelease(action.Drone, action.Destination)
					for _, action2 := range actions { //防止有的汽车仍然被某些action需要
						if action2.Car == action.Car && action2.ActionType == core.DroneCarToChargePoint && action2.Drone != action.Drone {
							if action.Time+float64(action.StartTime) != action2.Time+float64(action2.StartTime) { //结束时间不一样
								flag = false
							}
						}
					}
					if flag {
						agentRelease(action.Car, action.Destination)
						chargePoints[action.ChargePointId].Status = 0
					}
					action.Drone.RemainingPower = action.Drone.FullPower
				}
			case core.CarToChargePoint:
				{
					agentRelease(action.Car, action.Destination)
				}
			case core.DroneToChargePoint:
				{
					agentRelease(action.Drone, action.Destination)
					uavPowerRelease(action.Drone, action.CostPow)
				}
			}
		}
	}
	//电量更新
	//for _, agent := range agents {
	//	drone, isDrone := agent.(*core.Drone)
	//	if isDrone {
	//		if drone.RemainingPower-drone.GetSpeed()*float64(core.DecideTime) < 0 {
	//			drone.RemainingPower = 0
	//		} else {
	//			drone.RemainingPower = drone.RemainingPower - drone.GetSpeed()*float64(core.DecideTime)
	//		}
	//	}
	//}

	//更新当轮决策新上线和下线的智能体
	for _, agent := range allAgents {
		if i == 1 && agent.GetUpTime() == 0 {
			agents = append(agents, agent)
		}
		if float64((i-1)*core.DecideTime) < agent.GetUpTime() && agent.GetUpTime() <= float64(i*core.DecideTime) && agent.GetDate() == date {
			agents = append(agents, agent)
		}
		//查看是否到下线时间了 33 30 40(4)
		if float64((i-1)*core.DecideTime) < agent.GetDownTime() && agent.GetDownTime() <= float64(i*core.DecideTime) {
			removeIds = append(removeIds, agent.GetUUId())
		}
	}

	//
	sort.Slice(removeIndexs, func(i, j int) bool {
		return removeIndexs[i] > removeIndexs[j]
	})

	for _, index := range removeIndexs {
		if index >= 0 && index < len(actions) {
			if index == len(actions)-1 {
				actions = actions[:index]
			} else {
				actions = append(actions[:index], actions[index+1:]...)
			}

		}
	}
	for _, id := range removeIds {
		agents = removeElement(agents, uuidAgent[id], func(agent core.Agent, agent2 core.Agent) bool {
			return agent.GetUUId() == agent2.GetUUId()
		})
	}
	return actions
}

func uavPowerRelease(drone *core.Drone, pow float64) {
	drone.RemainingPower = drone.RemainingPower - pow
	if drone.RemainingPower < 0 {
		println("电量异常-----------------------------------------------------------------------------")
	}
}

// 删除指定值
func removeElement[T any](arr []T, element T, equalFunc func(T, T) bool) []T {
	j := 0
	for _, v := range arr {
		if !equalFunc(element, v) {
			arr[j] = v
			j++
		}
	}
	return arr[:j]
}

func agentRelease(agent core.Agent, position core.Position) {
	if agent != nil && agent.IsValid() {
		agent.SetPosition(position.X, position.Y)
		agent.SetStatus(core.Idle)
	}
}
