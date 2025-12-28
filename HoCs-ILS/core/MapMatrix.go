package core

import (
	"encoding/csv"
	"fmt"
	"github.com/google/uuid"
	"log"
	"os"
	"strconv"
)

// MapMatrix 表示地图矩阵结构体
type MapMatrix struct {
	AgentMap  [][]int // 智能体矩阵，1代表无人机，2代表工作人员，3代表汽车
	TaskMap   [][]int // 任务地图
	ChargeMap [][]int // 充电点地图
}

// LoadMapData 从文件读取数据并返回 mapData 和 Agent 集合
// csv格式
func LoadMapData(uavMapFile, workerMapFile, carMapFile, taskMapFile, chargeMapFile string) (*MapMatrix, []Agent, map[int]*TaskPoint, map[int]*ChargePoint, map[string]Agent, []bool, error) {
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
	agents := make([]Agent, 0)
	taskPoints := make(map[int]*TaskPoint)
	chargePoints := make(map[int]*ChargePoint)

	// Helper function to parse CSV files
	parseCSV := func(filePath string) ([][]string, error) {
		file, err := os.Open(filePath)
		if err != nil {
			return nil, fmt.Errorf("无法打开文件 %s: %v", filePath, err)
		}
		defer file.Close()
		reader := csv.NewReader(file)
		rows, err := reader.ReadAll()
		if err != nil {
			return nil, fmt.Errorf("无法读取文件 %s: %v", filePath, err)
		}
		return rows, nil
	}

	// 读取 UAV 数据
	rows, err := parseCSV(uavMapFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, err
	}
	for _, row := range rows[1:] { // 从第二行开始读取
		id := toInt(row[0])
		x := toInt(row[1])
		y := toInt(row[2])
		fullPower := toInt(row[4])
		uRget := toFloat(row[3])
		upTime := toFloat(row[6])
		downTime := toFloat(row[7])
		uPow := toInt(row[5])

		uav := &Drone{
			Base: Base{
				Id:       id,
				Position: Position{X: x, Y: y},
			},
			date:           1,
			UUID:           uuid.New().String(),
			IsPublisher:    false,
			RemainingPower: float64(uPow),
			UpTime:         upTime,
			FullPower:      float64(fullPower),
			URget:          uRget,
			DownTime:       downTime,
			NeedCharge:     false,
		}
		uuidAgent[uav.UUID] = uav
		agents = append(agents, uav)
		mapMatrix.AgentMap[y][x] = 1 // 1 表示无人机
	}

	// 读取 Worker 数据
	rows, err = parseCSV(workerMapFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, err
	}
	for _, row := range rows[1:] {
		id := toInt(row[0])
		x := toInt(row[1])
		y := toInt(row[2])
		wRget := toFloat(row[3])
		upTime := toFloat(row[4])
		downTime := toFloat(row[5])

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

	// 读取 Car 数据
	rows, err = parseCSV(carMapFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, err
	}
	for _, row := range rows[1:] {
		id := toInt(row[0])
		x := toInt(row[1])
		y := toInt(row[2])
		cRget := toFloat(row[3])
		upTime := toFloat(row[4])
		downTime := toFloat(row[5])
		chargePow := toFloat(row[6])

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
		uuidAgent[car.UUID] = car
		agents = append(agents, car)
		mapMatrix.AgentMap[y][x] = 3 // 3 表示汽车
	}
	dates[1] = true
	// 读取 Task 数据
	rows, err = parseCSV(taskMapFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, err
	}
	for _, row := range rows[1:] {
		id := toInt(row[0])
		x := toInt(row[1])
		y := toInt(row[2])
		taskPow := toFloat(row[3])

		taskPoint := TaskPoint{
			Base: Base{
				Id:       id,
				Position: Position{X: x, Y: y},
			},
			CostPow: taskPow,
		}
		taskPoints[taskPoint.Id] = &taskPoint
		mapMatrix.TaskMap[y][x] = 1 // 1 表示任务
	}

	// 读取 Charge 数据
	rows, err = parseCSV(chargeMapFile)
	if err != nil {
		return nil, nil, nil, nil, nil, nil, err
	}
	for _, row := range rows[1:] {
		id := toInt(row[0])
		x := toInt(row[1])
		y := toInt(row[2])

		chargePoint := ChargePoint{
			Base: Base{
				Id:       id,
				Position: Position{X: x, Y: y},
			},
		}
		chargePoints[chargePoint.Id] = &chargePoint
		mapMatrix.ChargeMap[y][x] = 1 // 1 表示充电点

	}
	return mapMatrix, agents, taskPoints, chargePoints, uuidAgent, dates, nil
}

func toInt(s string) int {
	value, _ := strconv.Atoi(s)
	return value
}

func toFloat(s string) float64 {
	value, _ := strconv.ParseFloat(s, 64)
	return value
}

func excelizeToInt(value string) int {
	// 将字符串转换为 int
	result, err := strconv.Atoi(value)
	if err != nil {
		log.Fatalf("无法将值转换为整数: %v", err)
	}
	return result
}
