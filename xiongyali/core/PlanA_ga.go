package core

import (
	"fmt"
	"math"
	"math/rand"
	"sort"
)

type gaTaskFitness struct {
	L          int
	DoneH      int
	Navg       float64
	Nmax       int
	NeedCharge int
}

type gaTaskIndividual struct {
	Picks   []int
	fitness gaTaskFitness
	valid   bool
}

func (e *PlanAEngine) selectTaskSubset(tasksList []*TaskPoint, workers []*Worker, drones []*Drone, horizon int) []*TaskPoint {
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
	best := runTaskSubsetGA(e, candidates, workers, drones, horizon, k, rng)
	return subsetFromPicks(candidates, best.Picks)
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

func runTaskSubsetGA(e *PlanAEngine, candidates []*TaskPoint, workers []*Worker, drones []*Drone, horizon int, k int, rng *rand.Rand) gaTaskIndividual {
	popSize := 30
	generations := 30
	stallLimit := 5
	pop := initTaskPopulation(candidates, k, popSize, rng)
	for i := range pop {
		pop[i] = evaluateTaskIndividual(e, pop[i], candidates, workers, drones, horizon)
	}
	best := bestTaskIndividual(pop)
	stall := 0

	for gen := 0; gen < generations; gen++ {
		next := make([]gaTaskIndividual, 0, popSize)
		next = append(next, best)

		attempt := 0
		accept := 0
		dupReject := 0
		seen := map[string]struct{}{}

		maxAttempts := popSize * 20
		for len(next) < popSize && attempt < maxAttempts {
			attempt++
			parentA := tournamentTaskSelect(pop, rng, 3)
			parentB := tournamentTaskSelect(pop, rng, 3)
			child := crossoverTaskSubset(parentA, parentB, candidates, k, rng)
			mutateTaskSubset(&child, candidates, k, rng, 0.3)
			child = evaluateTaskIndividual(e, child, candidates, workers, drones, horizon)
			if !child.valid {
				continue
			}
			key := taskKey(child)
			if _, ok := seen[key]; ok {
				dupReject++
				continue
			}
			seen[key] = struct{}{}
			next = append(next, child)
			accept++
		}

		pop = next
		newBest := bestTaskIndividual(pop)
		if betterTaskFitness(newBest.fitness, best.fitness) {
			best = newBest
			stall = 0
		} else {
			stall++
		}

		distinctL := map[int]int{}
		distinctDoneH := map[int]int{}
		distinctNeed := map[int]int{}
		for _, ind := range pop {
			distinctL[ind.fitness.L]++
			distinctDoneH[ind.fitness.DoneH]++
			distinctNeed[ind.fitness.NeedCharge]++
		}

		fmt.Printf("[GA][gen=%d] attempt=%d accept=%d dup=%d\n", gen, attempt, accept, dupReject)
		fmt.Printf("[GA][gen=%d] distinct L=%d DoneH=%d Need=%d\n", gen, len(distinctL), len(distinctDoneH), len(distinctNeed))

		if stall >= stallLimit {
			break
		}
	}

	return best
}

func initTaskPopulation(candidates []*TaskPoint, k int, popSize int, rng *rand.Rand) []gaTaskIndividual {
	pop := make([]gaTaskIndividual, 0, popSize)
	for len(pop) < popSize {
		picks := randomTaskPicks(len(candidates), k, rng)
		pop = append(pop, gaTaskIndividual{Picks: picks})
	}
	return pop
}

func randomTaskPicks(n int, k int, rng *rand.Rand) []int {
	idx := rng.Perm(n)
	return append([]int(nil), idx[:k]...)
}

func crossoverTaskSubset(a gaTaskIndividual, b gaTaskIndividual, candidates []*TaskPoint, k int, rng *rand.Rand) gaTaskIndividual {
	pickSet := make(map[int]struct{})
	child := make([]int, 0, k)
	cut := rng.Intn(len(a.Picks))
	for i := 0; i < cut; i++ {
		child = append(child, a.Picks[i])
		pickSet[a.Picks[i]] = struct{}{}
	}
	for _, v := range b.Picks {
		if len(child) >= k {
			break
		}
		if _, ok := pickSet[v]; ok {
			continue
		}
		child = append(child, v)
		pickSet[v] = struct{}{}
	}
	for len(child) < k {
		randIdx := rng.Intn(len(candidates))
		if _, ok := pickSet[randIdx]; ok {
			continue
		}
		child = append(child, randIdx)
		pickSet[randIdx] = struct{}{}
	}
	return gaTaskIndividual{Picks: child}
}

func mutateTaskSubset(ind *gaTaskIndividual, candidates []*TaskPoint, k int, rng *rand.Rand, rate float64) {
	if rng.Float64() > rate {
		return
	}
	if len(ind.Picks) == 0 {
		return
	}
	pickSet := make(map[int]struct{}, len(ind.Picks))
	for _, v := range ind.Picks {
		pickSet[v] = struct{}{}
	}
	replaceIdx := rng.Intn(len(ind.Picks))
	for {
		newIdx := rng.Intn(len(candidates))
		if _, ok := pickSet[newIdx]; ok {
			continue
		}
		delete(pickSet, ind.Picks[replaceIdx])
		ind.Picks[replaceIdx] = newIdx
		pickSet[newIdx] = struct{}{}
		return
	}
}

func evaluateTaskIndividual(e *PlanAEngine, ind gaTaskIndividual, candidates []*TaskPoint, workers []*Worker, drones []*Drone, horizon int) gaTaskIndividual {
	if len(ind.Picks) == 0 {
		ind.valid = false
		return ind
	}
	subset := subsetFromPicks(candidates, ind.Picks)
	omegaU := buildOmegaU(subset, drones)
	omegaW := e.buildOmegaW(subset, workers)
	costWT := e.buildCostWT(subset, workers, drones, omegaW, omegaU)
	assignedTasks, taskToWorker := buildTaskAssignment(costWT, e.cfg.BigM)
	if len(assignedTasks) == 0 {
		ind.valid = true
		ind.fitness = gaTaskFitness{L: 0}
		return ind
	}
	costUT := e.buildCostUT(subset, workers, drones, assignedTasks, taskToWorker)
	droneToTask := buildDroneToTaskIdxFromCost(assignedTasks, costUT, e.cfg.BigM)
	fitness := evaluateAssignments(subset, workers, drones, taskToWorker, droneToTask, horizon)
	ind.fitness = fitness
	ind.valid = true
	return ind
}

func evaluateAssignments(tasks []*TaskPoint, workers []*Worker, drones []*Drone, taskToWorker map[int]int, droneToTask map[int]int, horizon int) gaTaskFitness {
	l := 0
	doneH := 0
	nmax := 0
	nsum := 0
	needCharge := 0
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
		if nr > nmax {
			nmax = nr
		}
		if nr <= horizon {
			doneH++
		}
		nsum += nr
		dist := Distance(d.X, d.Y, t.X, t.Y)
		slack := d.RemainingPower - (dist + t.CostPow + t.ChargeDist)
		if slack < 0 {
			needCharge++
		}
	}
	navg := 0.0
	if l > 0 {
		navg = float64(nsum) / float64(l)
	}
	return gaTaskFitness{
		L:          l,
		DoneH:      doneH,
		Navg:       navg,
		Nmax:       nmax,
		NeedCharge: needCharge,
	}
}

func betterTaskFitness(a gaTaskFitness, b gaTaskFitness) bool {
	if a.L != b.L {
		return a.L > b.L
	}
	if a.DoneH != b.DoneH {
		return a.DoneH > b.DoneH
	}
	if a.Navg != b.Navg {
		return a.Navg < b.Navg
	}
	if a.Nmax != b.Nmax {
		return a.Nmax < b.Nmax
	}
	return a.NeedCharge < b.NeedCharge
}

func bestTaskIndividual(pop []gaTaskIndividual) gaTaskIndividual {
	best := pop[0]
	for i := 1; i < len(pop); i++ {
		if betterTaskFitness(pop[i].fitness, best.fitness) {
			best = pop[i]
		}
	}
	return best
}

func tournamentTaskSelect(pop []gaTaskIndividual, rng *rand.Rand, k int) gaTaskIndividual {
	best := pop[rng.Intn(len(pop))]
	for i := 1; i < k; i++ {
		cand := pop[rng.Intn(len(pop))]
		if betterTaskFitness(cand.fitness, best.fitness) {
			best = cand
		}
	}
	return best
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

func taskKey(ind gaTaskIndividual) string {
	picks := append([]int(nil), ind.Picks...)
	sort.Ints(picks)
	return fmt.Sprintf("%v", picks)
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
