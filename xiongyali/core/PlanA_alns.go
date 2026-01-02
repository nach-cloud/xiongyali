package core

import (
	"math"
	"math/rand"
)

type planState struct {
	tasks          []*TaskPoint
	taskToWorker   map[int]int
	droneToTaskIdx map[int]int
	finalDroneTask map[int]int
	droneCharge    map[int]struct{}
	droneWeights   map[int]float64
	carTarget      map[int]int
	forcedCharge   map[int]struct{}
}

type planScore struct {
	L        int
	DoneH    int
	Discount float64
}

func (e *PlanAEngine) optimizePlanALNS(tasksList []*TaskPoint, workers []*Worker, drones []*Drone, cars []*Car, chargePoints map[int]*ChargePoint, carTarget map[int]int, chargeList []*ChargePoint, taskPoints map[int]*TaskPoint, horizon int) *planState {
	candidates := buildTaskCandidates(tasksList, drones)
	if len(candidates) == 0 {
		return nil
	}
	k := minInt(len(candidates), len(workers), len(drones), e.cfg.TaskSubsetMax)
	if k <= 0 {
		return nil
	}

	rng := rand.New(rand.NewSource(int64(horizon*100 + len(candidates))))
	baseTasks := initPlanFromCandidates(candidates, k, rng)
	current := e.buildPlanFromTasks(subsetFromPicks(candidates, baseTasks), workers, drones, cars, taskPoints, carTarget, chargePoints, nil)
	if current == nil || len(current.tasks) == 0 {
		return nil
	}
	best := current
	bestScore := evaluatePlanScore(best, workers, drones, horizon)
	currentScore := bestScore

	iterations := 80
	gamma := 0.9
	temperature := 1.0
	for iter := 0; iter < iterations; iter++ {
		cand := destroyPlanState(current, len(drones), chargeList, rng)
		cand = repairPlanState(e, cand, candidates, k, workers, drones, cars, taskPoints, chargePoints, rng)
		if cand == nil {
			continue
		}
		candScore := evaluatePlanScore(cand, workers, drones, horizon)
		if betterPlanScore(candScore, bestScore) {
			best = cand
			bestScore = candScore
		}
		if betterPlanScore(candScore, currentScore) || acceptBySA(currentScore, candScore, temperature, rng) {
			current = cand
			currentScore = candScore
		}
		temperature *= gamma
		if temperature < 1e-3 {
			temperature = 1e-3
		}
	}

	return best
}

func buildTaskCandidates(tasksList []*TaskPoint, drones []*Drone) []*TaskPoint {
	candidates := make([]*TaskPoint, 0, len(tasksList))
	for _, t := range tasksList {
		if t == nil {
			continue
		}
		hasDrone := false
		for _, d := range drones {
			if feasibleDroneTask(d, t) {
				hasDrone = true
				break
			}
		}
		if hasDrone {
			candidates = append(candidates, t)
		}
	}
	return candidates
}

func initPlanFromCandidates(candidates []*TaskPoint, k int, rng *rand.Rand) []int {
	picks := rng.Perm(len(candidates))
	return append([]int(nil), picks[:k]...)
}

func destroyPlanState(plan *planState, droneCount int, chargeList []*ChargePoint, rng *rand.Rand) *planState {
	if plan == nil {
		return nil
	}
	cand := clonePlanState(plan)
	cand.tasks = destroyTasks(cand.tasks, rng)
	cand.forcedCharge = destroyChargeSet(cand.forcedCharge, droneCount, rng)
	cand.carTarget = destroyCarTargets(cand.carTarget, chargeList, rng)
	return cand
}

func destroyTasks(tasks []*TaskPoint, rng *rand.Rand) []*TaskPoint {
	if len(tasks) == 0 {
		return tasks
	}
	count := maxInt(1, len(tasks)/5)
	indices := rng.Perm(len(tasks))[:count]
	toRemove := make(map[int]struct{}, count)
	for _, idx := range indices {
		toRemove[idx] = struct{}{}
	}
	remaining := make([]*TaskPoint, 0, len(tasks)-count)
	for i, t := range tasks {
		if _, ok := toRemove[i]; ok {
			continue
		}
		remaining = append(remaining, t)
	}
	return remaining
}

func destroyChargeSet(forced map[int]struct{}, maxCount int, rng *rand.Rand) map[int]struct{} {
	newSet := make(map[int]struct{})
	for k := range forced {
		newSet[k] = struct{}{}
	}
	if maxCount == 0 {
		return newSet
	}
	if rng.Float64() < 0.5 {
		idx := rng.Intn(maxCount)
		if _, ok := newSet[idx]; ok {
			delete(newSet, idx)
		} else {
			newSet[idx] = struct{}{}
		}
	}
	return newSet
}

func destroyCarTargets(carTarget map[int]int, chargeList []*ChargePoint, rng *rand.Rand) map[int]int {
	newTarget := make(map[int]int)
	for k, v := range carTarget {
		newTarget[k] = v
	}
	if len(chargeList) == 0 || len(newTarget) == 0 {
		return newTarget
	}
	keys := make([]int, 0, len(newTarget))
	for k := range newTarget {
		keys = append(keys, k)
	}
	idx := keys[rng.Intn(len(keys))]
	newTarget[idx] = chargeList[rng.Intn(len(chargeList))].Id
	return newTarget
}

func repairPlanState(e *PlanAEngine, cand *planState, candidates []*TaskPoint, k int, workers []*Worker, drones []*Drone, cars []*Car, taskPoints map[int]*TaskPoint, chargePoints map[int]*ChargePoint, rng *rand.Rand) *planState {
	if cand == nil {
		return nil
	}
	taskSet := repairTasks(cand.tasks, candidates, k, rng)
	return e.buildPlanFromTasks(taskSet, workers, drones, cars, taskPoints, cand.carTarget, chargePoints, cand.forcedCharge)
}

func repairTasks(tasks []*TaskPoint, candidates []*TaskPoint, k int, rng *rand.Rand) []*TaskPoint {
	pickSet := make(map[int]struct{}, len(tasks))
	for _, t := range tasks {
		if t == nil {
			continue
		}
		for idx, cand := range candidates {
			if cand == t {
				pickSet[idx] = struct{}{}
				break
			}
		}
	}
	for len(pickSet) < k {
		idx := rng.Intn(len(candidates))
		pickSet[idx] = struct{}{}
	}
	picks := make([]int, 0, len(pickSet))
	for idx := range pickSet {
		picks = append(picks, idx)
	}
	return subsetFromPicks(candidates, picks)
}

func (e *PlanAEngine) buildPlanFromTasks(tasks []*TaskPoint, workers []*Worker, drones []*Drone, cars []*Car, taskPoints map[int]*TaskPoint, carTarget map[int]int, chargePoints map[int]*ChargePoint, forcedCharge map[int]struct{}) *planState {
	if len(tasks) == 0 {
		return nil
	}
	omegaU := buildOmegaU(tasks, drones)
	omegaW := e.buildOmegaW(tasks, workers)
	costWT := e.buildCostWT(tasks, workers, drones, omegaW, omegaU)
	assignedTasks, taskToWorker := buildTaskAssignment(costWT, e.cfg.BigM)
	if len(assignedTasks) == 0 {
		return nil
	}
	costUT := buildCostUTWithChargeMask(e, tasks, workers, drones, assignedTasks, taskToWorker, forcedCharge)
	droneToTaskIdx := buildDroneToTaskIdxFromCost(assignedTasks, costUT, e.cfg.BigM)
	finalDroneTask, droneCharge, droneWeights := e.buildDroneDecisionsPreview(drones, cars, tasks, taskPoints, droneToTaskIdx, carTarget, chargePoints)
	if forcedCharge != nil {
		for di := range forcedCharge {
			if di < 0 || di >= len(drones) {
				continue
			}
			delete(finalDroneTask, di)
			droneCharge[di] = struct{}{}
		}
	}
	return &planState{
		tasks:          tasks,
		taskToWorker:   taskToWorker,
		droneToTaskIdx: droneToTaskIdx,
		finalDroneTask: finalDroneTask,
		droneCharge:    droneCharge,
		droneWeights:   droneWeights,
		carTarget:      carTarget,
		forcedCharge:   forcedCharge,
	}
}

func buildCostUTWithChargeMask(e *PlanAEngine, tasksList []*TaskPoint, workers []*Worker, drones []*Drone, assignedTasks []int, taskToWorker map[int]int, forcedCharge map[int]struct{}) [][]float64 {
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
		if _, ok := forcedCharge[i]; ok {
			continue
		}
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

func evaluatePlanScore(plan *planState, workers []*Worker, drones []*Drone, horizon int) planScore {
	if plan == nil {
		return planScore{}
	}
	return evaluateAssignmentsScore(plan.tasks, workers, drones, plan.taskToWorker, plan.finalDroneTask, horizon)
}

func evaluateAssignmentsScore(tasks []*TaskPoint, workers []*Worker, drones []*Drone, taskToWorker map[int]int, droneToTask map[int]int, horizon int) planScore {
	l := 0
	doneH := 0
	discount := 0.0
	gamma := 0.9
	for di, tIdx := range droneToTask {
		if tIdx < 0 || tIdx >= len(tasks) {
			continue
		}
		d := drones[di]
		t := tasks[tIdx]
		wIdx, ok := taskToWorker[tIdx]
		if !ok {
			continue
		}
		w := workers[wIdx]
		if d == nil || w == nil || t == nil {
			continue
		}
		l++
		nr := nFinish(d, w, t)
		if nr <= horizon {
			doneH++
		}
		discount += math.Pow(gamma, float64(nr))
	}
	return planScore{
		L:        l,
		DoneH:    doneH,
		Discount: discount,
	}
}

func betterPlanScore(a planScore, b planScore) bool {
	if a.L != b.L {
		return a.L > b.L
	}
	if a.DoneH != b.DoneH {
		return a.DoneH > b.DoneH
	}
	return a.Discount > b.Discount
}

func acceptBySA(current planScore, cand planScore, temperature float64, rng *rand.Rand) bool {
	if temperature <= 0 {
		return false
	}
	delta := float64(cand.L-current.L) + float64(cand.DoneH-current.DoneH) + (cand.Discount - current.Discount)
	if delta >= 0 {
		return true
	}
	return rng.Float64() < math.Exp(delta/temperature)
}

func subsetFromPicks(candidates []*TaskPoint, picks []int) []*TaskPoint {
	subset := make([]*TaskPoint, 0, len(picks))
	for _, idx := range picks {
		if idx >= 0 && idx < len(candidates) {
			subset = append(subset, candidates[idx])
		}
	}
	return subset
}

func clonePlanState(plan *planState) *planState {
	if plan == nil {
		return nil
	}
	clone := *plan
	clone.tasks = append([]*TaskPoint(nil), plan.tasks...)
	clone.taskToWorker = cloneIntMap(plan.taskToWorker)
	clone.droneToTaskIdx = cloneIntMap(plan.droneToTaskIdx)
	clone.finalDroneTask = cloneIntMap(plan.finalDroneTask)
	clone.droneCharge = cloneIntSet(plan.droneCharge)
	clone.droneWeights = cloneFloatMap(plan.droneWeights)
	clone.carTarget = cloneIntMap(plan.carTarget)
	clone.forcedCharge = cloneIntSet(plan.forcedCharge)
	return &clone
}

func cloneIntMap(src map[int]int) map[int]int {
	dst := make(map[int]int, len(src))
	for k, v := range src {
		dst[k] = v
	}
	return dst
}

func cloneFloatMap(src map[int]float64) map[int]float64 {
	dst := make(map[int]float64, len(src))
	for k, v := range src {
		dst[k] = v
	}
	return dst
}

func cloneIntSet(src map[int]struct{}) map[int]struct{} {
	dst := make(map[int]struct{}, len(src))
	for k := range src {
		dst[k] = struct{}{}
	}
	return dst
}

func minInt(values ...int) int {
	if len(values) == 0 {
		return 0
	}
	min := values[0]
	for _, v := range values[1:] {
		if v < min {
			min = v
		}
	}
	return min
}

func maxInt(a int, b int) int {
	if a > b {
		return a
	}
	return b
}
