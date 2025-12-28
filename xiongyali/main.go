package main

import (
	"encoding/csv"
	"fmt"
	"os"
	"sort"
	"time"

	"github.com/yourname/xiongyali/core"
)

var agents []core.Agent
var taskPoints map[int]*core.TaskPoint
var chargePoints map[int]*core.ChargePoint
var uuidAgent map[string]core.Agent

// ============================
// Main entry for Plan-A algorithm
// ============================
//
// This main file is a cleaned-up version of the main() you pasted.
// It keeps your data loading / simulation loop, but replaces the previous
// execute(s, graph, ...) logic with PlanAEngine.Decide(...).
//
// Assumptions (based on your existing project):
//  - core.DataProcess, core.LoadMapData, core.UpdateResultInThirdColumn, core.QuChong exist.
//  - updateAgent(allAgents, step, dayIndex, actions) exists and updates agent states.
//  - chargeTask() exists and updates each TaskPoint.ChargePoint and TaskPoint.ChargeDist.
//
// If your project already has a main.go, you can:
//  - rename this file to main.go, or
//  - call runPlanA() from your existing main.

func main() {
	name := "result/4.11result/testData.csv"
	folders := core.DataProcess(name)
	f, err := os.Open(name)
	if err != nil {
		panic(err)
	}
	defer f.Close()
	reader := csv.NewReader(f)
	records, _ := reader.ReadAll()

	for _, folder := range folders {

		if core.QuChong(records, folder) {
			continue
		}

		fmt.Println(folder.Name+"文件已经开始, decideTime", core.DecideTime, "decideCount", core.DecideCount)
		runs := 10
		var sumResult float64
		var sumElapsed float64
		resultArr := make([]float64, 0, runs)

		for run := 1; run <= runs; run++ {
			start := time.Now()
			fmt.Println("第", run, "轮计算")

			agents = make([]core.Agent, 0)
			taskPoints = make(map[int]*core.TaskPoint)
			chargePoints = make(map[int]*core.ChargePoint)

			pre := "data/4.11dataCSV/" + folder.Name
			_, allAgents, allTaskPoints, allChargePoints, uuid, dates, err := core.LoadMapData(
				pre+"/uavMap.csv",
				pre+"/workerMap.csv",
				pre+"/carMap.csv",
				pre+"/taskMap.csv",
				pre+"/chargeMap.csv",
			)
			if err != nil {
				fmt.Println("Error loading map data:", err)
				return
			}
			uuidAgent = uuid

			for key, value := range allTaskPoints {
				taskPoints[key] = value
			}
			chargePoints = allChargePoints

			completeTaskCount := 0

			engine := core.NewPlanAEngine(core.PlanAConfig{
				TopLTasksPerWorker:    50,
				ChargeMeetWindowSteps: 2,
				BigM:                  1e9,
			})

			actions := make([]core.Action, 0)

			activeDays := 0.0
			for dayIndex, ok := range dates {
				if !ok {
					continue
				}
				activeDays++
				for step := 1; step <= core.DecideCount; step++ {
					fmt.Println("以下第", step, "轮决策：")

					actions = updateAgent(allAgents, step, dayIndex, actions)
					chargeTask()

					res := engine.Decide(step, dayIndex, taskPoints, chargePoints, agents)
					completeTaskCount += len(res.CompletedTaskIDs)
					actions = append(actions, res.Actions...)
				}

				agents = make([]core.Agent, 0)
				actions = make([]core.Action, 0)
				taskPoints = make(map[int]*core.TaskPoint)
				for key, value := range allTaskPoints {
					taskPoints[key] = value
				}
				for _, cp := range chargePoints {
					if cp != nil {
						cp.Status = 0
					}
				}
			}

			perResult := 0.0
			if activeDays > 0 {
				perResult = float64(completeTaskCount) / (float64(len(allTaskPoints)) * activeDays)
			}
			fmt.Println("完成率为", perResult)
			sumResult += perResult
			resultArr = append(resultArr, perResult)

			elapsed := time.Since(start).Seconds()
			sumElapsed += elapsed
			fmt.Println("本轮耗时", elapsed, "s")
		}

		avg := sumResult / float64(runs)
		fmt.Println("平均值：", avg)
		sort.Float64s(resultArr)
		maxFl := resultArr[len(resultArr)-1]
		avgElapsed := sumElapsed / float64(runs)

		records = core.UpdateResultInThirdColumn(records, name, folder, avg, maxFl, avgElapsed)
		out, _ := os.OpenFile(name, os.O_RDWR|os.O_TRUNC, 0666)
		defer out.Close()
		writer := csv.NewWriter(out)
		if err := writer.WriteAll(records); err != nil {
			fmt.Println(err)
		}
	}
}

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

func updateAgent(allAgents []core.Agent, i int, date int, actions []core.Action) []core.Action {
	removeIndexs := make([]int, 0)
	var removeIds []string
	for index, action := range actions {
		deadLine := action.Time + float64(action.StartTime)
		if deadLine <= float64(i*core.DecideTime) && date == action.Date {
			removeIndexs = append(removeIndexs, index)
			switch action.ActionType {
			case core.DroneToTask:
				agentRelease(action.Drone, action.Destination)
				uavPowerRelease(action.Drone, action.CostPow)
			case core.DroneWorkerToTask:
				agentRelease(action.Drone, action.Destination)
				agentRelease(action.Worker, action.Destination)
				uavPowerRelease(action.Drone, action.CostPow)
			case core.WorkerToTask:
				agentRelease(action.Worker, action.Destination)
			case core.DroneCarToChargePoint:
				flag := true
				agentRelease(action.Drone, action.Destination)
				for _, action2 := range actions {
					if action2.Car == action.Car && action2.ActionType == core.DroneCarToChargePoint && action2.Drone != action.Drone {
						if action.Time+float64(action.StartTime) != action2.Time+float64(action2.StartTime) {
							flag = false
						}
					}
				}
				if flag {
					agentRelease(action.Car, action.Destination)
					chargePoints[action.ChargePointId].Status = 0
				}
				action.Drone.RemainingPower = action.Drone.FullPower
			case core.CarToChargePoint:
				agentRelease(action.Car, action.Destination)
			case core.DroneToChargePoint:
				agentRelease(action.Drone, action.Destination)
				uavPowerRelease(action.Drone, action.CostPow)
			}
		}
	}

	for _, agent := range allAgents {
		if i == 1 && agent.GetUpTime() == 0 {
			agents = append(agents, agent)
		}
		if float64((i-1)*core.DecideTime) < agent.GetUpTime() && agent.GetUpTime() <= float64(i*core.DecideTime) && agent.GetDate() == date {
			agents = append(agents, agent)
		}
		if float64((i-1)*core.DecideTime) < agent.GetDownTime() && agent.GetDownTime() <= float64(i*core.DecideTime) {
			removeIds = append(removeIds, agent.GetUUId())
		}
	}

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
