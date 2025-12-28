package core

import (
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
	return &PlanAEngine{
		cfg:              cfg,
		droneCommitTask:  make(map[string]int),
		workerCommitTask: make(map[string]int),
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
	// -------------------------
	// Step 1: build idle sets (online filtering assumed already reflected by your updateAgent)
	// -------------------------
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
			if v != nil && v.Status == Idle {
				cars = append(cars, v)
			}
		}
	}

	// Early exits
	if len(taskPoints) == 0 {
		// no tasks: maybe charge drones if beneficial; keep it simple
		return e.decideOnlyCharging(step, date, drones, cars, chargePoints, taskPoints)
	}
	if len(workers) == 0 || len(drones) == 0 {
		// cannot complete tasks; only charging / repositioning
		return e.decideOnlyCharging(step, date, drones, cars, chargePoints, taskPoints)
	}

	// -------------------------
	// Step 2: candidate sets ΩW(j), ΩU(x)
	// -------------------------
	tasksList := make([]*TaskPoint, 0, len(taskPoints))
	for _, t := range taskPoints {
		if t != nil {
			tasksList = append(tasksList, t)
		}
	}

	// ΩU(x): feasible drones per task
	omegaU := make(map[int][]int) // taskID -> indices of feasible drones
	for ti, t := range tasksList {
		_ = ti
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

	// ΩW(j): top-L tasks by worker distance/steps
	omegaW := make(map[int][]int) // workerIndex -> task indices (in tasksList)
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

	// -------------------------
	// Step 3: Worker–Task Hungarian (min cost)
	// CostWT(j, x) = min_{i feasible} N_finish(i,j,x)
	// -------------------------
	// Build full task index set [0..len(tasksList)-1]
	// but allow +INF where not in ΩW or no feasible drone.
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
	// Solve assignment: worker -> taskIndex (or -1)
	wtAssign := hungarianMin(costWT, e.cfg.BigM)
	// wtAssign len = m, value is task index or -1

	// Build assigned task list (unique) + mapping taskIndex -> workerIndex
	assignedTasks := make([]int, 0)
	taskToWorker := make(map[int]int)
	for j, tIdx := range wtAssign {
		if tIdx < 0 || tIdx >= n {
			continue
		}
		if costWT[j][tIdx] >= e.cfg.BigM/2 {
			continue
		}
		if _, exists := taskToWorker[tIdx]; exists {
			continue
		}
		assignedTasks = append(assignedTasks, tIdx)
		taskToWorker[tIdx] = j
	}
	if len(assignedTasks) == 0 {
		// no feasible worker-task match -> charging only
		return e.decideOnlyCharging(step, date, drones, cars, chargePoints, taskPoints)
	}

	// -------------------------
	// Step 4: Drone–Task Hungarian (min cost) among assigned tasks
	// CostUT(i, x(j)) = N_finish(i,j,x)
	// -------------------------
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
	utAssign := hungarianMin(costUT, e.cfg.BigM) // drone -> assignedTasksIndex

	// Build candidate triples (droneIndex -> taskIndex)
	droneToTaskIdx := make(map[int]int)
	for i, k := range utAssign {
		if k < 0 || k >= tN {
			continue
		}
		if costUT[i][k] >= e.cfg.BigM/2 {
			continue
		}
		droneToTaskIdx[i] = assignedTasks[k]
	}

	// -------------------------
	// Step 5: Car initial targets (nearest chargepoint)
	// -------------------------
	carTarget := make(map[int]int) // carIndex -> chargePointId
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
		bestID := chargeList[0].Id
		best := math.MaxFloat64
		for _, ch := range chargeList {
			dist := Distance(c.X, c.Y, ch.X, ch.Y)
			if dist < best {
				best = dist
				bestID = ch.Id
			}
		}
		carTarget[ci] = bestID
	}

	// -------------------------
	// Step 6: Drone decision (task vs charge) + build charge intents
	// -------------------------
	// final task set (droneIndex -> taskIdx)
	finalDroneTask := make(map[int]int)
	// charge intents (droneIndex -> (carIndex, chargeID))
	type chargeIntent struct {
		carIdx   int
		chargeID int
		meetN    int
	}
	droneCharge := make(map[int]chargeIntent)

	// Helper: best meet with cars on their initial targets
	bestMeet := func(d *Drone) (best chargeIntent, ok bool) {
		best.meetN = math.MaxInt
		best.carIdx = -1
		best.chargeID = -1
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
			if meet < best.meetN {
				best.meetN = meet
				best.carIdx = ci
				best.chargeID = chID
			}
		}
		if best.carIdx >= 0 {
			return best, true
		}
		return best, false
	}

	// commitment-aware task preference
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

	// First, handle drones with candidate task
	for di, d := range drones {
		if d == nil {
			continue
		}

		// If committed and still feasible, keep it.
		if tid, ok := committedTaskForDrone(d); ok {
			// Find its worker if possible; if not, we still move toward task.
			// We do not create DroneWorkerToTask unless we also have a worker assigned;
			// your simulation can handle DroneToTask/WorkerToTask if desired.
			// Here, we keep it in finalDroneTask to keep direction.
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
		//w := workers[taskToWorker[tIdx]]

		// 情况A（你给的最新版方案）：
		// 无人机一旦在候选任务匹配中拿到 (i,j,x)，就直接选择执行任务。
		// 充电只在“没有候选任务”的情况下考虑（情况B）。
		finalDroneTask[di] = tIdx
		e.droneCommitTask[d.UUID] = t.Id
	}

	// Now handle drones without candidate task: decide charge if ΔF>0 and quick meet
	for di, d := range drones {
		if d == nil {
			continue
		}
		if _, alreadyTask := finalDroneTask[di]; alreadyTask {
			continue
		}
		if _, alreadyCharge := droneCharge[di]; alreadyCharge {
			continue
		}
		// Potential unlock
		deltaF := deltaFeasibleTasks(d, tasksList)
		intent, ok := bestMeet(d)
		if ok && deltaF > 0 && intent.meetN <= e.cfg.ChargeMeetWindowSteps {
			droneCharge[di] = intent
		}
	}

	// -------------------------
	// Step 7: Final Car–Charge assignment using Benefit(k,y)
	// -------------------------
	// Build D_ch
	dronesWantCharge := make([]int, 0)
	for di := range droneCharge {
		dronesWantCharge = append(dronesWantCharge, di)
	}

	// If no drones want charge or no cars/charges, skip
	carToChargeID := make(map[int]int) // carIndex -> chargeID
	if len(dronesWantCharge) > 0 && len(cars) > 0 && len(chargeList) > 0 {
		// Build benefit matrix (cars x chargePoints)
		cN := len(cars)
		pN := len(chargeList)
		// maximize Benefit -> minimize negative benefit
		benefitCost := make([][]float64, cN)
		for ci := 0; ci < cN; ci++ {
			benefitCost[ci] = make([]float64, pN)
			for pi := 0; pi < pN; pi++ {
				benefitCost[ci][pi] = 0
			}
		}
		for ci, c := range cars {
			for pi, ch := range chargeList {
				if c == nil || ch == nil {
					benefitCost[ci][pi] = 0
					continue
				}
				count := 0
				for _, di := range dronesWantCharge {
					d := drones[di]
					if d == nil {
						continue
					}
					meetN := nMeet(d, c, ch)
					if meetN <= e.cfg.ChargeMeetWindowSteps {
						count++
					}
				}
				// cost = -benefit (minimization)
				benefitCost[ci][pi] = -float64(count)
			}
		}
		carAssign := hungarianMin(benefitCost, e.cfg.BigM)
		for ci, pi := range carAssign {
			if pi >= 0 && pi < len(chargeList) {
				carToChargeID[ci] = chargeList[pi].Id
			}
		}
	}

	// -------------------------
	// Step 8: Build Actions (Match^{q}{uwt} and Match^{q}{ucch})
	// -------------------------
	actions := make([]Action, 0)
	completedIDs := make([]int, 0)

	// (A) Task actions: only when both drone and worker are assigned to SAME task.
	// We build DroneWorkerToTask actions for those pairs.
	// IMPORTANT: This follows your current style: once we decide DroneWorkerToTask,
	// we mark statuses Busy, remove task from taskPoints, and count completion.
	// If your simulator counts completion later, move this logic outside.
	usedWorkers := make(map[int]struct{})
	for di, tIdx := range finalDroneTask {
		d := drones[di]
		t := tasksList[tIdx]
		wIdx := taskToWorker[tIdx]
		w := workers[wIdx]
		if d == nil || w == nil || t == nil {
			continue
		}
		if _, ok := usedWorkers[wIdx]; ok {
			continue
		}
		usedWorkers[wIdx] = struct{}{}

		// Build DroneWorkerToTask action
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

		// Mark completion now (consistent with your old execute())
		delete(taskPoints, t.Id)
		completedIDs = append(completedIDs, t.Id)
	}

	// (B) Charge actions: DroneCarToChargePoint for drones that chose charging
	for di, intent := range droneCharge {
		d := drones[di]
		if d == nil || len(cars) == 0 {
			continue
		}
		// Choose car:
		// - If we have a final car->charge assignment, prefer the car that goes to that charge.
		// - Otherwise use the original best intent car.
		chosenCarIdx := intent.carIdx
		chosenChargeID := intent.chargeID
		// If car is assigned to some chargepoint, use that point as destination
		if chID, ok := carToChargeID[chosenCarIdx]; ok {
			chosenChargeID = chID
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

	return PlanAResult{Actions: actions, CompletedTaskIDs: completedIDs}
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
			// nearest charge for this car
			chID, _ := FoundNearChargePoisition(c.X, c.Y, chargePoints)
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
