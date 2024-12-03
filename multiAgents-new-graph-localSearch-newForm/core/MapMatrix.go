package core

import (
	"fmt"
	"github.com/google/uuid"
	"github.com/xuri/excelize/v2"
	"log"
	"strconv"
)

// MapMatrix 表示地图矩阵结构体
type MapMatrix struct {
	AgentMap  [][]int // 智能体矩阵，1代表无人机，2代表工作人员，3代表汽车
	TaskMap   [][]int // 任务地图
	ChargeMap [][]int // 充电点地图
}

// LoadMapData 从文件读取数据并返回 mapData 和 Agent 集合
func LoadMapData(uavMapFile, workerMapFile, carMapFile, taskMapFile, chargeMapFile string) (*MapMatrix, []Agent, map[int]*TaskPoint, map[int]*ChargePoint, map[string]Agent, []bool, error) {
	// 创建一个 MapMatrix 结构体
	width, height := 100, 100 // 假设地图的宽度和高度为 100，可以根据需要调整
	uuidAgent := make(map[string]Agent)
	mapMatrix := &MapMatrix{
		AgentMap:  make([][]int, height),
		TaskMap:   make([][]int, height),
		ChargeMap: make([][]int, height),
	}

	for i := range mapMatrix.AgentMap {
		mapMatrix.AgentMap[i] = make([]int, width)
		mapMatrix.TaskMap[i] = make([]int, width)
		mapMatrix.ChargeMap[i] = make([]int, width)
	}
	dates := make([]bool, 50)
	// 创建一个 Agent 列表
	agents := make([]Agent, 0)
	taskPoints := make(map[int]*TaskPoint) // 读取 uavMap 数据
	chargePoints := make(map[int]*ChargePoint)
	uavFile, err := excelize.OpenFile(uavMapFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("无法打开无人机表格文件: %v", err)
	}

	uavSheet := uavFile.GetSheetList()[0]
	rows, err := uavFile.GetRows(uavSheet)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("无法读取无人机表格数据: %v", err)
	}

	for _, row := range rows[1:] { // 从第二行开始读取
		id := excelizeToInt(row[0])
		//date := excelizeToInt(row[1])
		x := excelizeToInt(row[1])
		y := excelizeToInt(row[2])
		FullPower := excelizeToInt(row[4])
		//FullPower := 20
		//uRget, _ := strconv.ParseFloat(row[5], 64)
		uRget, _ := strconv.ParseFloat(row[3], 64)
		upTime, _ := strconv.ParseFloat(row[6], 64)
		downTime, _ := strconv.ParseFloat(row[7], 64)
		uPow := excelizeToInt(row[5])
		//uPow := 18
		//remainingPower := excelizeToInt(row[1])

		uav := &Drone{
			Base: Base{
				Id:       id,
				Position: Position{X: x, Y: y},
			},
			date:           1,
			UUID:           uuid.New().String(),
			IsPublisher:    false,
			RemainingPower: float64(uPow), //剩余电量移动距离
			UpTime:         upTime,
			FullPower:      float64(FullPower),
			URget:          float64(uRget),
			DownTime:       downTime,
			NeedCharge:     false,
		}
		//dates[date] = true
		uuidAgent[uav.UUID] = uav
		agents = append(agents, uav)
		mapMatrix.AgentMap[y][x] = 1 // 1 表示无人机
	}

	// 读取 workerMap 数据
	workerFile, err := excelize.OpenFile(workerMapFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("无法打开工作人员表格文件: %v", err)
	}

	workerSheet := workerFile.GetSheetList()[0]
	rows, err = workerFile.GetRows(workerSheet)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("无法读取工作人员表格数据: %v", err)
	}

	for _, row := range rows[1:] { // 从第二行开始读取
		id := excelizeToInt(row[0])
		//date := excelizeToInt(row[1])
		x := excelizeToInt(row[1])
		y := excelizeToInt(row[2])

		wRget, _ := strconv.ParseFloat(row[3], 64)
		upTime, _ := strconv.ParseFloat(row[4], 64)
		downTime, _ := strconv.ParseFloat(row[5], 64)
		//dates[date] = true
		worker := &Worker{
			Base: Base{
				Id:       id,
				Position: Position{X: x, Y: y},
			},
			IsPublisher: false,
			UUID:        uuid.New().String(),
			WRget:       wRget,
			UpTime:      upTime,
			DownTime:    downTime,
			date:        1,
		}
		uuidAgent[worker.UUID] = worker
		agents = append(agents, worker)
		mapMatrix.AgentMap[y][x] = 2 // 2 表示工作人员
	}

	// 读取 carMap 数据
	carFile, err := excelize.OpenFile(carMapFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("无法打开汽车表格文件: %v", err)
	}

	carSheet := carFile.GetSheetList()[0]
	rows, err = carFile.GetRows(carSheet)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("无法读取汽车表格数据: %v", err)
	}

	for _, row := range rows[1:] { // 从第二行开始读取
		id := excelizeToInt(row[0])
		//date := excelizeToInt(row[1])
		x := excelizeToInt(row[1])
		y := excelizeToInt(row[2])
		//cRget := excelizeToInt(row[4])
		cRget, _ := strconv.ParseFloat(row[3], 64)
		upTime, _ := strconv.ParseFloat(row[4], 64)
		downTime, _ := strconv.ParseFloat(row[5], 64)
		chargePow, _ := strconv.ParseFloat(row[6], 64)
		car := &Car{
			Base: Base{
				Id:       id,
				Position: Position{X: x, Y: y},
			},
			IsPublisher: false,
			UUID:        uuid.New().String(),
			CRget:       cRget,
			UpTime:      upTime,
			DownTime:    downTime,
			ChargePow:   chargePow,
			date:        1,
		}
		dates[1] = true
		uuidAgent[car.UUID] = car
		agents = append(agents, car)
		mapMatrix.AgentMap[y][x] = 3 // 3 表示汽车
	}

	// 读取 taskMap 数据
	taskFile, err := excelize.OpenFile(taskMapFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("无法打开任务表格文件: %v", err)
	}

	taskSheet := taskFile.GetSheetList()[0]
	rows, err = taskFile.GetRows(taskSheet)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("无法读取任务表格数据: %v", err)
	}

	for _, row := range rows[1:] { // 从第二行开始读取
		id := excelizeToInt(row[0])
		x := excelizeToInt(row[1])
		y := excelizeToInt(row[2])
		taskPow, _ := strconv.ParseFloat(row[3], 64)
		taskpoint := TaskPoint{
			Base: Base{
				Id:       id,
				Position: Position{X: x, Y: y},
			},
			Drones:  make([]DroneCollection, 0),
			Workers: make([]Worker, 0),
			Cars:    make([]Car, 0),
			UpTime:  0,
			CostPow: taskPow,
		}

		taskPoints[taskpoint.Id] = &taskpoint
		mapMatrix.TaskMap[y][x] = 1 // 1 表示任务
	}

	// 读取 chargeMap 数据
	chargeFile, err := excelize.OpenFile(chargeMapFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("无法打开充电点表格文件: %v", err)
	}

	chargeSheet := chargeFile.GetSheetList()[0]
	rows, err = chargeFile.GetRows(chargeSheet)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, fmt.Errorf("无法读取充电点表格数据: %v", err)
	}

	for _, row := range rows[1:] { // 从第二行开始读取
		id := excelizeToInt(row[0])
		x := excelizeToInt(row[1])
		y := excelizeToInt(row[2])
		//upTime := excelizeToInt(row[3])
		chargePoint := ChargePoint{
			Base: Base{
				Id:       id,
				Position: Position{X: x, Y: y},
			},
			UpTime: 0,
		}
		chargePoints[chargePoint.Id] = &chargePoint
		mapMatrix.ChargeMap[y][x] = 1 // 1 表示充电点
	}

	return mapMatrix, agents, taskPoints, chargePoints, uuidAgent, dates, nil
}

func excelizeToInt(value string) int {
	// 将字符串转换为 int
	result, err := strconv.Atoi(value)
	if err != nil {
		log.Fatalf("无法将值转换为整数: %v", err)
	}
	return result
}
