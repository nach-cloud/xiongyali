package core

import (
	"fmt"
	"math"
	"sort"
)

// ============================
// Plan A (rolling, no future-worker prediction)
// ============================
//
// This file provides the algorithm core for “方案A”.
// It is intentionally self-contained (includes a Hungarian solver) and only
// depends on your existing core types (Drone/Worker/Car/TaskPoint/ChargePoint,
// DecideTime, Distance, computeDecideTime, etc.).
//
// IMPORTANT:
// 1) Hard constraints (1)(2)(3)(4) are NOT modified here.
//    - Feasible(i,x) uses (3) exactly.
// 2) We do NOT try to predict which workers will be idle after charging.
//    Rolling re-optimization at each decision step handles that.
// 3) Equality checks for (1)(2) are assumed handled by your simulation/move
//    update (e.g., snapping to target coordinates when within one step).

// PlanAConfig holds tunable engineering knobs.
// These are NOT “power thresholds”; they are only complexity/window knobs.
type PlanAConfig struct {
	// TopLTasksPerWorker limits candidate tasks per worker (ΩW(j)).
	// Typical: 5~30.
	TopLTasksPerWorker int

	// ChargeMeetWindowSteps is Hc in the description: we only consider charging
	// “worth it” if a drone can meet some car within Hc steps.
	// Typical: 1 or 2.
	ChargeMeetWindowSteps int

	// BigM is the “infinite” cost used in assignment matrices.
	BigM float64

	// TaskSubsetMax limits GA-selected tasks per step.
	TaskSubsetMax int
}

// PlanAEngine keeps light state across decision steps.
// We only store short “commitments” to reduce thrashing.
type PlanAEngine struct {
	cfg PlanAConfig

	// droneCommitTask maps drone UUID -> committed task id (if any).
	// Commitment lasts until task disappears (completed) or becomes infeasible.
	droneCommitTask map[string]int

	// workerCommitTask maps worker UUID -> committed task id.
	workerCommitTask map[string]int

	// carPlan tracks planned charge targets across decision steps.
	carPlan map[string]*CarPlanEntry
}

// NewPlanAEngine constructs the algorithm engine.
func NewPlanAEngine(cfg PlanAConfig) *PlanAEngine {
	if cfg.TopLTasksPerWorker <= 0 {
		cfg.TopLTasksPerWorker = 15
	}
	if cfg.ChargeMeetWindowSteps <= 0 {
		cfg.ChargeMeetWindowSteps = 2
	}
	if cfg.BigM <= 0 {
		cfg.BigM = 1e9
	}
	if cfg.TaskSubsetMax <= 0 {
		cfg.TaskSubsetMax = 20
	}
	return &PlanAEngine{
		cfg:              cfg,
		droneCommitTask:  make(map[string]int),
		workerCommitTask: make(map[string]int),
		carPlan:          make(map[string]*CarPlanEntry),
	}
}

// PlanAResult is the decision output of one decision step.
//   - Actions: to be appended to your action list.
//   - CompletedTaskIDs: tasks that are decided as “DroneWorkerToTask” this step.
//     (Keep consistent with your current accounting approach.)
type PlanAResult struct {
	Actions          []Action
	CompletedTaskIDs []int
}

type CarPlanEntry struct {
	TargetChargeID int
	Fixed          bool
	LastUpdateStep int
}

// Decide runs “方案A” for one decision step.
// Inputs:
//   - step: 1..DecideCount
//   - date: day index
//   - taskPoints: CURRENT remaining tasks (you already pass a mutable map)
//   - chargePoints: all charge points (mutable map)
//   - agents: all agents at current step (your slice)
//
// NOTE: This function does NOT call updateAgent() or chargeTask();
// it only decides assignments and returns Actions.
func (e *PlanAEngine) Decide(step int, date int, taskPoints map[int]*TaskPoint, chargePoints map[int]*ChargePoint, agents []Agent) PlanAResult {
	drones, workers, cars := collectAgents(agents)
	carTarget, chargeList := e.updateCarPlan(step, cars, chargePoints)
	if len(taskPoints) == 0 || len(workers) == 0 || len(drones) == 0 {
		return e.decideOnlyCharging(step, date, drones, cars, chargePoints, taskPoints)
	}

	tasksList := buildTasksList(taskPoints)
	tasksList = e.selectTaskSubset(tasksList, workers, drones, horizon)
	if len(tasksList) == 0 {
		return e.decideOnlyCharging(step, date, drones, cars, chargePoints, taskPoints)
	}
	omegaU := buildOmegaU(tasksList, drones)
	omegaW := e.buildOmegaW(tasksList, workers)
	costWT := e.buildCostWT(tasksList, workers, drones, omegaW, omegaU)
	assignedTasks, taskToWorker := buildTaskAssignment(costWT, e.cfg.BigM)
	if len(assignedTasks) == 0 {
		return e.decideOnlyCharging(step, date, drones, cars, chargePoints, taskPoints)
	}

	costUT := e.buildCostUT(tasksList, workers, drones, assignedTasks, taskToWorker)
	droneToTaskIdx := buildDroneToTaskIdxFromCost(assignedTasks, costUT, e.cfg.BigM)
	finalDroneTask, droneCharge, droneWeights := e.buildDroneDecisions(drones, cars, tasksList, taskPoints, droneToTaskIdx, carTarget, chargePoints)
	fmt.Printf("[EXEC] taskCand=%d taskMap=%d finalTask=%d charge=%d\n", len(tasksList), len(droneToTaskIdx), len(finalDroneTask), len(droneCharge))
	carToChargeID, droneToCar := e.assignCarsToCharges(droneCharge, droneWeights, drones, cars, chargeList, carTarget)
	actions, completedIDs := e.buildActions(step, date, drones, workers, cars, tasksList, taskPoints, chargePoints, finalDroneTask, droneCharge, taskToWorker, carToChargeID, droneToCar)

	return PlanAResult{Actions: actions, CompletedTaskIDs: completedIDs}
}

func collectAgents(agents []Agent) ([]*Drone, []*Worker, []*Car) {
	drones := make([]*Drone, 0)
	workers := make([]*Worker, 0)
	cars := make([]*Car, 0)
	for _, a := range agents {
		if a == nil {
			continue
		}
		switch v := a.(type) {
		case *Drone:
			if v != nil && v.Status == Idle {
				drones = append(drones, v)
			}
		case *Worker:
			if v != nil && v.Status == Idle {
				workers = append(workers, v)
			}
		case *Car:
			if v != nil {
				cars = append(cars, v)
			}
		}
	}
	return drones, workers, cars
}

func buildTasksList(taskPoints map[int]*TaskPoint) []*TaskPoint {
	tasksList := make([]*TaskPoint, 0, len(taskPoints))
	for _, t := range taskPoints {
		if t != nil {
			tasksList = append(tasksList, t)
		}
	}
	return tasksList
}

func buildOmegaU(tasksList []*TaskPoint, drones []*Drone) map[int][]int {
	omegaU := make(map[int][]int)
	for _, t := range tasksList {
		feasible := make([]int, 0)
		for di, d := range drones {
			if d == nil {
				continue
			}
			if feasibleDroneTask(d, t) {
				feasible = append(feasible, di)
			}
		}
		omegaU[t.Id] = feasible
	}
	return omegaU
}

func (e *PlanAEngine) buildOmegaW(tasksList []*TaskPoint, workers []*Worker) map[int][]int {
	omegaW := make(map[int][]int)
	for wIdx, w := range workers {
		if w == nil {
			continue
		}
		type cand struct {
			tIdx  int
			steps int
		}
		cands := make([]cand, 0, len(tasksList))
		for tIdx, t := range tasksList {
			if t == nil {
				continue
			}
			s := stepsTo(w.GetPosition(), t.Position, w.WRget)
			cands = append(cands, cand{tIdx: tIdx, steps: s})
		}
		sort.Slice(cands, func(i, j int) bool { return cands[i].steps < cands[j].steps })
		L := e.cfg.TopLTasksPerWorker
		if L > len(cands) {
			L = len(cands)
		}
		picked := make([]int, 0, L)
		for i := 0; i < L; i++ {
			picked = append(picked, cands[i].tIdx)
		}
		omegaW[wIdx] = picked
	}
	return omegaW
}

func (e *PlanAEngine) buildCostWT(tasksList []*TaskPoint, workers []*Worker, drones []*Drone, omegaW map[int][]int, omegaU map[int][]int) [][]float64 {
	m := len(workers)
	n := len(tasksList)
	costWT := make([][]float64, m)
	for j := 0; j < m; j++ {
		costWT[j] = make([]float64, n)
		for x := 0; x < n; x++ {
			costWT[j][x] = e.cfg.BigM
		}
	}
	for j := 0; j < m; j++ {
		allowed := make(map[int]struct{}, len(omegaW[j]))
		for _, tIdx := range omegaW[j] {
			allowed[tIdx] = struct{}{}
		}
		w := workers[j]
		for x := 0; x < n; x++ {
			if _, ok := allowed[x]; !ok {
				continue
			}
			t := tasksList[x]
			feasibleD := omegaU[t.Id]
			if len(feasibleD) == 0 {
				continue
			}
			best := math.MaxFloat64
			for _, di := range feasibleD {
				d := drones[di]
				nf := nFinish(d, w, t)
				if float64(nf) < best {
					best = float64(nf)
				}
			}
			costWT[j][x] = best
		}
	}
	return costWT
}

func buildTaskAssignment(costWT [][]float64, bigM float64) ([]int, map[int]int) {
	m := len(costWT)
	n := 0
	if m > 0 {
		n = len(costWT[0])
	}
	wtAssign := hungarianMin(costWT, bigM)
	assignedTasks := make([]int, 0)
	taskToWorker := make(map[int]int)
	for j, tIdx := range wtAssign {
		if tIdx < 0 || tIdx >= n {
			continue
		}
		if costWT[j][tIdx] >= bigM/2 {
			continue
		}
		if _, exists := taskToWorker[tIdx]; exists {
			continue
		}
		assignedTasks = append(assignedTasks, tIdx)
		taskToWorker[tIdx] = j
	}
	return assignedTasks, taskToWorker
}

func buildDroneToTaskIdxFromCost(assignedTasks []int, costUT [][]float64, bigM float64) map[int]int {
	tN := len(assignedTasks)
	utAssign := hungarianMin(costUT, bigM)
	droneToTaskIdx := make(map[int]int)
	for i, k := range utAssign {
		if k < 0 || k >= tN {
			continue
		}
		if costUT[i][k] >= bigM/2 {
			continue
		}
		droneToTaskIdx[i] = assignedTasks[k]
	}
	return droneToTaskIdx
}

func (e *PlanAEngine) buildCostUT(tasksList []*TaskPoint, workers []*Worker, drones []*Drone, assignedTasks []int, taskToWorker map[int]int) [][]float64 {
	dN := len(drones)
	tN := len(assignedTasks)
	costUT := make([][]float64, dN)
	for i := 0; i < dN; i++ {
		costUT[i] = make([]float64, tN)
		for k := 0; k < tN; k++ {
			costUT[i][k] = e.cfg.BigM
		}
	}
	for i := 0; i < dN; i++ {
		d := drones[i]
		for k := 0; k < tN; k++ {
			tIdx := assignedTasks[k]
			t := tasksList[tIdx]
			if !feasibleDroneTask(d, t) {
				continue
			}
			w := workers[taskToWorker[tIdx]]
			costUT[i][k] = float64(nFinish(d, w, t))
		}
	}
	return costUT
}

func (e *PlanAEngine) updateCarPlan(step int, cars []*Car, chargePoints map[int]*ChargePoint) (map[int]int, []*ChargePoint) {
	carTarget := make(map[int]int)
	chargeList := make([]*ChargePoint, 0, len(chargePoints))
	for _, ch := range chargePoints {
		if ch != nil {
			chargeList = append(chargeList, ch)
		}
	}
	for ci, c := range cars {
		if c == nil || len(chargeList) == 0 {
			continue
		}
		entry := e.ensureCarPlanEntry(c, chargePoints, chargeList)
		entry.Fixed = c.Status != Idle
		entry.LastUpdateStep = step
		carTarget[ci] = entry.TargetChargeID
	}
	return carTarget, chargeList
}

func (e *PlanAEngine) ensureCarPlanEntry(car *Car, chargePoints map[int]*ChargePoint, chargeList []*ChargePoint) *CarPlanEntry {
	entry, ok := e.carPlan[car.UUID]
	if !ok {
		entry = &CarPlanEntry{}
		e.carPlan[car.UUID] = entry
	}
	if entry.TargetChargeID == 0 {
		entry.TargetChargeID = nearestChargeID(car, chargeList)
		return entry
	}
	if _, exists := chargePoints[entry.TargetChargeID]; !exists {
		entry.TargetChargeID = nearestChargeID(car, chargeList)
	}
	return entry
}

func nearestChargeID(car *Car, chargeList []*ChargePoint) int {
	bestID := chargeList[0].Id
	best := math.MaxFloat64
	for _, ch := range chargeList {
		dist := Distance(car.X, car.Y, ch.X, ch.Y)
		if dist < best {
			best = dist
			bestID = ch.Id
		}
	}
	return bestID
}

func (e *PlanAEngine) buildDroneDecisions(drones []*Drone, cars []*Car, tasksList []*TaskPoint, taskPoints map[int]*TaskPoint, droneToTaskIdx map[int]int, carTarget map[int]int, chargePoints map[int]*ChargePoint) (map[int]int, map[int]struct{}, map[int]float64) {
	finalDroneTask := make(map[int]int)
	droneCharge := make(map[int]struct{})
	droneWeights := make(map[int]float64)

	bestMeet := func(d *Drone) (int, bool) {
		best := math.MaxInt
		for ci, c := range cars {
			chID, has := carTarget[ci]
			if !has {
				continue
			}
			ch := chargePoints[chID]
			if ch == nil {
				continue
			}
			meet := nMeet(d, c, ch)
			if meet < best {
				best = meet
			}
		}
		if best < math.MaxInt {
			return best, true
		}
		return 0, false
	}

	committedTaskForDrone := func(d *Drone) (int, bool) {
		if d == nil {
			return 0, false
		}
		tid, ok := e.droneCommitTask[d.UUID]
		if !ok {
			return 0, false
		}
		if _, exists := taskPoints[tid]; !exists {
			delete(e.droneCommitTask, d.UUID)
			return 0, false
		}
		return tid, true
	}

	for di, d := range drones {
		if d == nil {
			continue
		}

		if tid, ok := committedTaskForDrone(d); ok {
			for tIdx, t := range tasksList {
				if t != nil && t.Id == tid {
					if feasibleDroneTask(d, t) {
						finalDroneTask[di] = tIdx
					} else {
						delete(e.droneCommitTask, d.UUID)
					}
					break
				}
			}
			continue
		}

		tIdx, hasTask := droneToTaskIdx[di]
		if !hasTask {
			continue
		}
		t := tasksList[tIdx]
		if feasibleDroneTask(d, t) {
			finalDroneTask[di] = tIdx
			e.droneCommitTask[d.UUID] = t.Id
		}
	}

	for di, d := range drones {
		if d == nil {
			continue
		}
		if _, alreadyTask := finalDroneTask[di]; alreadyTask {
			continue
		}
		deltaF := float64(deltaFeasibleTasks(d, tasksList))
		meetN, ok := bestMeet(d)
		if ok && deltaF > 0 && meetN <= e.cfg.ChargeMeetWindowSteps {
			droneCharge[di] = struct{}{}
			droneWeights[di] = deltaF
		}
	}

	return finalDroneTask, droneCharge, droneWeights
}

func (e *PlanAEngine) assignCarsToCharges(droneCharge map[int]struct{}, droneWeights map[int]float64, drones []*Drone, cars []*Car, chargeList []*ChargePoint, carTarget map[int]int) (map[int]int, map[int]int) {
	carToChargeID := make(map[int]int)
	droneToCar := make(map[int]int)
	if len(droneCharge) == 0 || len(cars) == 0 || len(chargeList) == 0 {
		return carToChargeID, droneToCar
	}

	for iter := 0; iter < 3; iter++ {
		targetPoints := chargePointByID(chargeList)
		droneToCar = assignDronesToCars(droneCharge, drones, cars, targetPoints, carTarget)
		changed := updateCarTargets(droneToCar, droneWeights, drones, cars, chargeList, carTarget, e.carPlan)
		if !changed {
			break
		}
	}

	for ci, chID := range carTarget {
		carToChargeID[ci] = chID
		if ci >= 0 && ci < len(cars) && cars[ci] != nil {
			entry := e.carPlan[cars[ci].UUID]
			if entry != nil {
				entry.TargetChargeID = chID
			}
		}
	}
	return carToChargeID, droneToCar
}

func chargePointByID(chargeList []*ChargePoint) map[int]*ChargePoint {
	chargeByID := make(map[int]*ChargePoint, len(chargeList))
	for _, ch := range chargeList {
		if ch != nil {
			chargeByID[ch.Id] = ch
		}
	}
	return chargeByID
}

func assignDronesToCars(droneCharge map[int]struct{}, drones []*Drone, cars []*Car, chargeByID map[int]*ChargePoint, carTarget map[int]int) map[int]int {
	droneToCar := make(map[int]int)
	for di := range droneCharge {
		d := drones[di]
		if d == nil {
			continue
		}
		bestCar := -1
		bestMeet := math.MaxInt
		for ci, c := range cars {
			if c == nil {
				continue
			}
			chID, ok := carTarget[ci]
			if !ok {
				continue
			}
			ch := chargeByID[chID]
			if ch == nil {
				continue
			}
			meet := nMeet(d, c, ch)
			if meet < bestMeet {
				bestMeet = meet
				bestCar = ci
			}
		}
		if bestCar >= 0 {
			droneToCar[di] = bestCar
		}
	}
	return droneToCar
}

func updateCarTargets(droneToCar map[int]int, droneWeights map[int]float64, drones []*Drone, cars []*Car, chargeList []*ChargePoint, carTarget map[int]int, carPlan map[string]*CarPlanEntry) bool {
	changed := false
	for ci, c := range cars {
		if c == nil {
			continue
		}
		entry := carPlan[c.UUID]
		if entry != nil && entry.Fixed {
			continue
		}
		assignedDrones := make([]int, 0)
		for di, carIdx := range droneToCar {
			if carIdx == ci {
				assignedDrones = append(assignedDrones, di)
			}
		}
		if len(assignedDrones) == 0 {
			continue
		}

		bestID := carTarget[ci]
		bestCost := math.Inf(1)
		for _, ch := range chargeList {
			if ch == nil {
				continue
			}
			cost := 0.0
			for _, di := range assignedDrones {
				d := drones[di]
				if d == nil {
					continue
				}
				meetN := nMeet(d, c, ch)
				weight := droneWeights[di]
				cost += weight * float64(meetN)
			}
			if cost < bestCost {
				bestCost = cost
				bestID = ch.Id
			}
		}
		if bestID != carTarget[ci] {
			carTarget[ci] = bestID
			changed = true
		}
	}
	return changed
}

func (e *PlanAEngine) buildActions(step int, date int, drones []*Drone, workers []*Worker, cars []*Car, tasksList []*TaskPoint, taskPoints map[int]*TaskPoint, chargePoints map[int]*ChargePoint, finalDroneTask map[int]int, droneCharge map[int]struct{}, taskToWorker map[int]int, carToChargeID map[int]int, droneToCar map[int]int) ([]Action, []int) {
	actions := make([]Action, 0)
	completedIDs := make([]int, 0)

	usedWorkers := make(map[int]struct{})
	for di, tIdx := range finalDroneTask {
		d := drones[di]
		t := tasksList[tIdx]
		wIdx, ok := taskToWorker[tIdx]
		if !ok {
			continue
		}
		w := workers[wIdx]
		if d == nil || w == nil || t == nil {
			continue
		}
		if _, ok := usedWorkers[wIdx]; ok {
			continue
		}
		usedWorkers[wIdx] = struct{}{}

		distU := Distance(d.X, d.Y, t.X, t.Y)
		distW := Distance(w.X, w.Y, t.X, t.Y)
		tu := computeDecideTime(distU / d.URget)
		tw := computeDecideTime(distW / w.WRget)
		tt := tu
		if tw > tt {
			tt = tw
		}
		costPow := distU + t.CostPow

		a := Action{
			Drone:       d,
			Worker:      w,
			TaskID:      t.Id,
			StartTime:   step * DecideTime,
			Time:        tt,
			CostPow:     costPow,
			Destination: t.Position,
			ActionType:  DroneWorkerToTask,
			Date:        date,
		}
		actions = append(actions, a)
		d.Status = Busy
		w.Status = Busy

		delete(taskPoints, t.Id)
		completedIDs = append(completedIDs, t.Id)
	}

	for di := range droneCharge {
		d := drones[di]
		if d == nil || len(cars) == 0 {
			continue
		}
		chosenCarIdx, ok := droneToCar[di]
		if !ok {
			continue
		}
		chosenChargeID, ok := carToChargeID[chosenCarIdx]
		if !ok {
			continue
		}
		ch := chargePoints[chosenChargeID]
		if ch == nil {
			continue
		}
		c := cars[chosenCarIdx]
		if c == nil {
			continue
		}

		distU := Distance(d.X, d.Y, ch.X, ch.Y)
		distC := Distance(c.X, c.Y, ch.X, ch.Y)
		tu := computeDecideTime(distU / d.URget)
		tc := computeDecideTime(distC / c.CRget)
		tt := tu
		if tc > tt {
			tt = tc
		}

		a := Action{
			Car:           c,
			Drone:         d,
			ChargePointId: ch.Id,
			StartTime:     step * DecideTime,
			Time:          tt,
			Destination:   ch.Position,
			ActionType:    DroneCarToChargePoint,
			Date:          date,
		}
		actions = append(actions, a)
		c.Status = Busy
		d.Status = Busy
		ch.Status = 1
	}

	return actions, completedIDs
}

// decideOnlyCharging is a fallback when tasks cannot be completed this step.
func (e *PlanAEngine) decideOnlyCharging(step int, date int, drones []*Drone, cars []*Car, chargePoints map[int]*ChargePoint, taskPoints map[int]*TaskPoint) PlanAResult {
	actions := make([]Action, 0)
	if len(drones) == 0 || len(cars) == 0 || len(chargePoints) == 0 {
		return PlanAResult{Actions: actions}
	}
	// simple: for each drone, if charging unlocks tasks (if any exist) and can meet a car quickly, send it.
	chargeList := make([]*ChargePoint, 0, len(chargePoints))
	for _, ch := range chargePoints {
		if ch != nil {
			chargeList = append(chargeList, ch)
		}
	}
	if len(chargeList) == 0 {
		return PlanAResult{Actions: actions}
	}
	for _, d := range drones {
		if d == nil {
			continue
		}
		// Meet any car at its nearest chargepoint (greedy)
		bestCar := (*Car)(nil)
		bestCh := (*ChargePoint)(nil)
		bestN := math.MaxInt
		for _, c := range cars {
			if c == nil {
				continue
			}
			chID := e.carPlanChargeTarget(c, chargePoints, chargeList)
			ch := chargePoints[chID]
			if ch == nil {
				continue
			}
			meet := nMeet(d, c, ch)
			if meet < bestN {
				bestN = meet
				bestCar = c
				bestCh = ch
			}
		}
		if bestCar == nil || bestCh == nil {
			continue
		}
		if bestN > e.cfg.ChargeMeetWindowSteps {
			continue
		}
		// if there are no tasks, charging is optional; we still allow it.
		// build action
		distU := Distance(d.X, d.Y, bestCh.X, bestCh.Y)
		distC := Distance(bestCar.X, bestCar.Y, bestCh.X, bestCh.Y)
		tu := computeDecideTime(distU / d.URget)
		tc := computeDecideTime(distC / bestCar.CRget)
		tt := tu
		if tc > tt {
			tt = tc
		}
		a := Action{
			Car:           bestCar,
			Drone:         d,
			ChargePointId: bestCh.Id,
			StartTime:     step * DecideTime,
			Time:          tt,
			Destination:   bestCh.Position,
			ActionType:    DroneCarToChargePoint,
			Date:          date,
		}
		actions = append(actions, a)
		bestCar.Status = Busy
		d.Status = Busy
		bestCh.Status = 1
	}
	return PlanAResult{Actions: actions}
}

func (e *PlanAEngine) carPlanChargeTarget(car *Car, chargePoints map[int]*ChargePoint, chargeList []*ChargePoint) int {
	if car == nil || len(chargeList) == 0 {
		return 0
	}
	entry := e.carPlan[car.UUID]
	if entry == nil {
		return nearestChargeID(car, chargeList)
	}
	if _, ok := chargePoints[entry.TargetChargeID]; ok {
		return entry.TargetChargeID
	}
	return nearestChargeID(car, chargeList)
}

// feasibleDroneTask implements constraint (3) using your data model.
// Equivalent to:
//
//	dist(u, task) + task.CostPow + task.ChargeDist <= uPow
func feasibleDroneTask(d *Drone, t *TaskPoint) bool {
	if d == nil || t == nil {
		return false
	}
	dist := Distance(d.X, d.Y, t.X, t.Y)
	need := dist + t.CostPow + t.ChargeDist
	return d.RemainingPower >= need
}

// deltaFeasibleTasks computes ΔF_i = |F_i(Full)| - |F_i(Current)|.
// Uses only constraint (3) feasibility, no worker prediction.
func deltaFeasibleTasks(d *Drone, tasks []*TaskPoint) int {
	if d == nil {
		return 0
	}
	full := 0
	cur := 0
	for _, t := range tasks {
		if t == nil {
			continue
		}
		dist := Distance(d.X, d.Y, t.X, t.Y)
		need := dist + t.CostPow + t.ChargeDist
		if d.FullPower >= need {
			full++
		}
		if d.RemainingPower >= need {
			cur++
		}
	}
	return full - cur
}

// stepsTo computes ceil(dist / (speed * DecideTime)).
// speed is “per minute”, DecideTime is “minutes”.
func stepsTo(from Position, to Position, speed float64) int {
	if speed <= 0 {
		return math.MaxInt / 4
	}
	dist := Distance(from.X, from.Y, to.X, to.Y)
	stepDist := speed * float64(DecideTime)
	if stepDist <= 0 {
		return math.MaxInt / 4
	}
	return int(math.Ceil(dist / stepDist))
}

// nFinish returns N_finish(i,j,x) in steps.
func nFinish(d *Drone, w *Worker, t *TaskPoint) int {
	nu := stepsTo(d.Position, t.Position, d.URget)
	nw := stepsTo(w.Position, t.Position, w.WRget)
	if nu > nw {
		return nu
	}
	return nw
}

// nMeet returns N_meet(i,k,y) in steps.
func nMeet(d *Drone, c *Car, ch *ChargePoint) int {
	nu := stepsTo(d.Position, ch.Position, d.URget)
	nc := stepsTo(c.Position, ch.Position, c.CRget)
	if nu > nc {
		return nu
	}
	return nc
}

// ============================
// Hungarian algorithm (minimization) for rectangular matrices
// ============================
// Returns assignment for each row: assigned column index, or -1.
// This implementation pads to square internally.

func hungarianMin(cost [][]float64, bigM float64) []int {
	nRows := len(cost)
	nCols := 0
	for i := range cost {
		if len(cost[i]) > nCols {
			nCols = len(cost[i])
		}
	}
	n := nRows
	if nCols > n {
		n = nCols
	}
	// build square matrix a[n][n]
	a := make([][]float64, n)
	for i := 0; i < n; i++ {
		a[i] = make([]float64, n)
		for j := 0; j < n; j++ {
			a[i][j] = bigM
		}
	}
	for i := 0; i < nRows; i++ {
		for j := 0; j < len(cost[i]); j++ {
			a[i][j] = cost[i][j]
		}
	}

	// potentials
	u := make([]float64, n+1)
	v := make([]float64, n+1)
	p := make([]int, n+1)
	way := make([]int, n+1)
	for i := 1; i <= n; i++ {
		p[0] = i
		j0 := 0
		minv := make([]float64, n+1)
		used := make([]bool, n+1)
		for j := 1; j <= n; j++ {
			minv[j] = math.Inf(1)
			used[j] = false
		}
		used[0] = true
		for {
			used[j0] = true
			i0 := p[j0]
			delta := math.Inf(1)
			j1 := 0
			for j := 1; j <= n; j++ {
				if used[j] {
					continue
				}
				cur := a[i0-1][j-1] - u[i0] - v[j]
				if cur < minv[j] {
					minv[j] = cur
					way[j] = j0
				}
				if minv[j] < delta {
					delta = minv[j]
					j1 = j
				}
			}
			for j := 0; j <= n; j++ {
				if used[j] {
					u[p[j]] += delta
					v[j] -= delta
				} else {
					minv[j] -= delta
				}
			}
			j0 = j1
			if p[j0] == 0 {
				break
			}
		}
		// augmenting
		for {
			j1 := way[j0]
			p[j0] = p[j1]
			j0 = j1
			if j0 == 0 {
				break
			}
		}
	}
	// p[j] matched row for column j
	ans := make([]int, nRows)
	for i := 0; i < nRows; i++ {
		ans[i] = -1
	}
	// build assignment: row -> col
	for j := 1; j <= n; j++ {
		i := p[j]
		if i >= 1 && i <= nRows {
			col := j - 1
			if col < nCols {
				// if assigned to padded column, ignore
				if col < len(cost[i-1]) && cost[i-1][col] < bigM/2 {
					ans[i-1] = col
				}
			}
		}
	}
	return ans
}
