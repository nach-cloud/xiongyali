package core

import (
	"math"
	"math/rand"
)

type planScore struct {
	L        int
	DoneH    int
	Discount float64
}

func (e *PlanAEngine) selectTaskSubsetALNS(tasksList []*TaskPoint, workers []*Worker, drones []*Drone, horizon int) []*TaskPoint {
	candidates := buildTaskCandidates(tasksList, drones)
	if len(candidates) == 0 {
		return nil
	}
	k := minInt(len(candidates), len(workers), len(drones), e.cfg.TaskSubsetMax)
	if k <= 0 {
		return nil
	}
	if len(candidates) <= k {
		return candidates
	}

	rng := rand.New(rand.NewSource(int64(horizon*100 + len(candidates))))
	best := initPlanFromCandidates(candidates, k, rng)
	bestScore := evaluateSubset(e, candidates, best, workers, drones, horizon)
	current := best
	currentScore := bestScore

	iterations := 80
	gamma := 0.9
	temperature := 1.0
	for iter := 0; iter < iterations; iter++ {
		cand := destroyPlan(current, rng)
		cand = repairPlan(cand, candidates, k, rng)
		candScore := evaluateSubset(e, candidates, cand, workers, drones, horizon)
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

	return subsetFromPicks(candidates, best)
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

func destroyPlan(plan []int, rng *rand.Rand) []int {
	if len(plan) == 0 {
		return plan
	}
	count := maxInt(1, len(plan)/5)
	indices := rng.Perm(len(plan))[:count]
	destroyed := make([]int, 0, len(plan)-count)
	toRemove := make(map[int]struct{}, count)
	for _, idx := range indices {
		toRemove[idx] = struct{}{}
	}
	for i, v := range plan {
		if _, ok := toRemove[i]; ok {
			continue
		}
		destroyed = append(destroyed, v)
	}
	return destroyed
}

func repairPlan(plan []int, candidates []*TaskPoint, k int, rng *rand.Rand) []int {
	pickSet := make(map[int]struct{}, len(plan))
	for _, v := range plan {
		pickSet[v] = struct{}{}
	}
	for len(plan) < k {
		idx := rng.Intn(len(candidates))
		if _, ok := pickSet[idx]; ok {
			continue
		}
		plan = append(plan, idx)
		pickSet[idx] = struct{}{}
	}
	return plan
}

func evaluateSubset(e *PlanAEngine, candidates []*TaskPoint, picks []int, workers []*Worker, drones []*Drone, horizon int) planScore {
	subset := subsetFromPicks(candidates, picks)
	omegaU := buildOmegaU(subset, drones)
	omegaW := e.buildOmegaW(subset, workers)
	costWT := e.buildCostWT(subset, workers, drones, omegaW, omegaU)
	assignedTasks, taskToWorker := buildTaskAssignment(costWT, e.cfg.BigM)
	if len(assignedTasks) == 0 {
		return planScore{}
	}
	costUT := e.buildCostUT(subset, workers, drones, assignedTasks, taskToWorker)
	droneToTask := buildDroneToTaskIdxFromCost(assignedTasks, costUT, e.cfg.BigM)
	return evaluateAssignmentsScore(subset, workers, drones, taskToWorker, droneToTask, horizon)
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
