package main

import (
    "encoding/csv"
    "fmt"
    "os"
    "sort"
    "strconv"
    "time"

    "./core" // NOTE: adjust import path to your project layout (e.g. "yourmod/core")
)

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
    // Example: keep your previous iteration selection.
    iterations := []int{107}
    for iteration := 0; iteration < 1; iteration++ {
        name := "data/4.11data/testRealDataIter" + strconv.Itoa(iterations[iteration]) + ".csv"

        folders := core.DataProcess(name)
        f, err := os.Open(name)
        if err != nil {
            panic(err)
        }
        defer f.Close()
        reader := csv.NewReader(f)
        records, _ := reader.ReadAll()

        for _, folder := range folders {
            core.DecideTime = folder.DecideTime
            core.DecideCount = folder.DecideCount

            // Skip if already has results
            if core.QuChong(records, folder) {
                continue
            }

            fmt.Println(folder.Name, "文件已经开始, decideTime", core.DecideTime, "decideCount", core.DecideCount)

            runs := 10
            var sumResult float64
            var sumElapsed float64
            resultArr := make([]float64, 0, runs)

            for run := 1; run <= runs; run++ {
                start := time.Now()
                fmt.Println("第", run, "轮计算")

                // --- Load one scenario ---
                pre := "./../7.8dataCSV/" + folder.Name
                _, allAgents, allTaskPoints, allChargePoints, _, dates, err := core.LoadMapData(
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

                // taskPoints is a working copy (remaining tasks)
                taskPoints := make(map[int]*core.TaskPoint)
                for key, value := range allTaskPoints {
                    taskPoints[key] = value
                }
                chargePoints := allChargePoints

                completeTaskCount := 0

                // Build Plan-A engine (keeps small commitment state across steps)
                engine := core.NewPlanAEngine(core.PlanAConfig{
                    TopLTasksPerWorker:     15,
                    ChargeMeetWindowSteps:  2,
                    BigM:                   1e9,
                })

                // actions buffer used by updateAgent()
                actions := make([]core.Action, 0)

                activeDays := 0.0
                for dayIndex, ok := range dates {
                    if !ok {
                        continue
                    }
                    activeDays++
                    for step := 1; step <= core.DecideCount; step++ {
                        fmt.Println("以下第", step, "轮决策：")

                        // 1) update agent positions / power / statuses based on past actions
                        actions = updateAgent(allAgents, step, dayIndex, actions)

                        // 2) update each task's nearest chargepoint (TaskPoint.ChargePoint/ChargeDist)
                        chargeTask()

                        // 3) decide actions with Plan-A
                        res := engine.Decide(step, dayIndex, taskPoints, chargePoints, allAgents)

                        // PlanAEngine already deletes tasks from taskPoints for DroneWorkerToTask.
                        // Keep your existing accounting method:
                        completeTaskCount += len(res.CompletedTaskIDs)

                        // Append new actions (to be executed by updateAgent in subsequent steps)
                        actions = append(actions, res.Actions...)
                    }

                    // Reset day state for next day
                    actions = make([]core.Action, 0)
                    // restore tasks for next day
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

            // write result back to CSV
            records = core.UpdateResultInThirdColumn(records, name, folder, avg, maxFl, avgElapsed)
            out, _ := os.OpenFile(name, os.O_RDWR|os.O_TRUNC, 0666)
            defer out.Close()
            writer := csv.NewWriter(out)
            if err := writer.WriteAll(records); err != nil {
                fmt.Println(err)
            }
        }
    }
}
