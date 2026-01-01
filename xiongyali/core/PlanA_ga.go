package core

import (
	"fmt"
	"math"
	"math/rand"
	"strings"
)

type seedTriples struct {
	taskOrder   []int
	seedWperm   []int
	seedDperm   []int
	droneToTask map[int]int
}

func (s seedTriples) len() int {
	return len(s.taskOrder)
}

func buildSeedTriples(assignedTasks []int, taskToWorker map[int]int, costUT [][]float64, bigM float64) seedTriples {
	tN := len(assignedTasks)
	utAssign := hungarianMin(costUT, bigM)
	droneToTask := make(map[int]int)
	for i, k := range utAssign {
		if k < 0 || k >= tN {
			continue
		}
		if costUT[i][k] >= bigM/2 {
			continue
		}
		droneToTask[i] = assignedTasks[k]
	}
	taskToDrone := make(map[int]int)
	for di, taskIdx := range droneToTask {
		taskToDrone[taskIdx] = di
	}

	taskOrder := make([]int, 0)
	seedW := make([]int, 0)
	seedD := make([]int, 0)
	for _, tIdx := range assignedTasks {
		workerIdx, ok := taskToWorker[tIdx]
		if !ok {
			continue
		}
		droneIdx, ok := taskToDrone[tIdx]
		if !ok {
			continue
		}
		taskOrder = append(taskOrder, tIdx)
		seedW = append(seedW, workerIdx)
		seedD = append(seedD, droneIdx)
	}

	return seedTriples{
		taskOrder:   taskOrder,
		seedWperm:   seedW,
		seedDperm:   seedD,
		droneToTask: droneToTask,
	}
}

type gaFitness struct {
	Exec     int
	DoneH    int
	SlackMin float64
	Nmax     int
	Navg     float64
	Esum     float64
}

type gaIndividual struct {
	Wperm   []int
	Dperm   []int
	Mask    []bool
	fitness gaFitness
	valid   bool
}

type gaPrecompute struct {
	tasks       []int
	drones      []int
	workers     []int
	droneIndex  map[int]int
	workerIndex map[int]int
	eneed       [][]float64
	feasible    [][]bool
	td          [][]int
	tw          [][]int
	horizon     int
	dpow        []float64
}

func (e *PlanAEngine) runTripleGA(seed seedTriples, tasksList []*TaskPoint, workers []*Worker, drones []*Drone, chargeList []*ChargePoint, horizon int, rng *rand.Rand) (map[int]int, map[int]int) {
	precompute := buildGAPrecompute(seed, tasksList, workers, drones, chargeList, horizon)
	if len(precompute.tasks) == 0 {
		return seed.droneToTask, buildTaskToWorker(seed.taskOrder, seed.seedWperm)
	}

	popSize := 30
	generations := 20
	stallLimit := 5
	pop := initPopulation(seed, precompute, rng, popSize)
	best := bestIndividual(pop)
	stall := 0

	for gen := 0; gen < generations; gen++ {
		next := make([]gaIndividual, 0, popSize)
		next = append(next, best)

		attempt := 0
		accept := 0
		dropRepair := 0
		dropInvalid := 0
		dupReject := 0
		seen := map[string]struct{}{}

		maxAttempts := popSize * 20
		for len(next) < popSize && attempt < maxAttempts {
			attempt++
			parentA := tournamentSelect(pop, rng, 3)
			parentB := tournamentSelect(pop, rng, 3)
			childW := pmx(parentA.Wperm, parentB.Wperm, rng)
			childD := pmx(parentA.Dperm, parentB.Dperm, rng)
			childM := maskCrossover(parentA.Mask, parentB.Mask, rng)
			ensureAtLeastOneTrue(childM, rng)
			child := gaIndividual{Wperm: childW, Dperm: childD, Mask: childM}
			if !repairDronePermMasked(child.Dperm, child.Mask, seed.seedDperm, precompute) {
				dropRepair++
				continue
			}
			mutateIndividual(&child, precompute, rng, 0.2, 0.2, 0.2)
			child = evaluateIndividual(child, precompute)
			if !child.valid {
				dropInvalid++
				continue
			}
			key := indKey(child)
			if _, ok := seen[key]; ok {
				dupReject++
				continue
			}
			seen[key] = struct{}{}
			next = append(next, child)
			accept++
		}

		pop = next
		newBest := bestIndividual(pop)
		if betterFitness(newBest.fitness, best.fitness) {
			best = newBest
			stall = 0
		} else {
			stall++
		}

		distinctDoneH := map[int]int{}
		distinctNmax := map[int]int{}
		distinctNavg := map[float64]int{}
		distinctExec := map[int]int{}
		for _, ind := range pop {
			distinctDoneH[ind.fitness.DoneH]++
			distinctNmax[ind.fitness.Nmax]++
			distinctNavg[ind.fitness.Navg]++
			distinctExec[ind.fitness.Exec]++
		}

		fmt.Printf("[GA][gen=%d] attempt=%d accept=%d dropRepair=%d dropInvalid=%d dup=%d\n",
			gen, attempt, accept, dropRepair, dropInvalid, dupReject)
		fmt.Printf("[GA][gen=%d] distinct DoneH=%d Nmax=%d Navg=%d\n",
			gen, len(distinctDoneH), len(distinctNmax), len(distinctNavg))
		fmt.Printf("[GA][gen=%d] distinct Exec=%d\n", gen, len(distinctExec))

		if stall >= stallLimit {
			break
		}
	}

	return buildDroneToTaskMasked(seed.taskOrder, best.Dperm, best.Mask), buildTaskToWorkerMasked(seed.taskOrder, best.Wperm, best.Mask)
}

func buildGAPrecompute(seed seedTriples, tasksList []*TaskPoint, workers []*Worker, drones []*Drone, chargeList []*ChargePoint, horizon int) gaPrecompute {
	taskOrder := append([]int(nil), seed.taskOrder...)
	droneList := append([]int(nil), seed.seedDperm...)
	workerList := append([]int(nil), seed.seedWperm...)

	droneIndex := make(map[int]int)
	for i, di := range droneList {
		droneIndex[di] = i
	}
	workerIndex := make(map[int]int)
	for i, wi := range workerList {
		workerIndex[wi] = i
	}

	eneed := make([][]float64, len(droneList))
	feasible := make([][]bool, len(droneList))
	td := make([][]int, len(droneList))
	dpow := make([]float64, len(droneList))
	for dIdx, droneID := range droneList {
		d := drones[droneID]
		eneed[dIdx] = make([]float64, len(taskOrder))
		feasible[dIdx] = make([]bool, len(taskOrder))
		td[dIdx] = make([]int, len(taskOrder))
		if d != nil {
			dpow[dIdx] = d.RemainingPower
		}
		for r, tIdx := range taskOrder {
			t := tasksList[tIdx]
			if d == nil || t == nil {
				eneed[dIdx][r] = math.Inf(1)
				feasible[dIdx][r] = false
				td[dIdx][r] = math.MaxInt / 4
				continue
			}
			dist := Distance(d.X, d.Y, t.X, t.Y)
			need := dist + t.CostPow + t.ChargeDist
			eneed[dIdx][r] = need
			feasible[dIdx][r] = d.RemainingPower >= need
			td[dIdx][r] = stepsTo(d.Position, t.Position, d.URget)
		}
	}

	tw := make([][]int, len(workerList))
	for wIdx, workerID := range workerList {
		w := workers[workerID]
		tw[wIdx] = make([]int, len(taskOrder))
		for r, tIdx := range taskOrder {
			t := tasksList[tIdx]
			if w == nil || t == nil {
				tw[wIdx][r] = math.MaxInt / 4
				continue
			}
			tw[wIdx][r] = stepsTo(w.Position, t.Position, w.WRget)
		}
	}

	return gaPrecompute{
		tasks:       taskOrder,
		drones:      droneList,
		workers:     workerList,
		droneIndex:  droneIndex,
		workerIndex: workerIndex,
		eneed:       eneed,
		feasible:    feasible,
		td:          td,
		tw:          tw,
		horizon:     horizon,
		dpow:        dpow,
	}
}

func initPopulation(seed seedTriples, precompute gaPrecompute, rng *rand.Rand, popSize int) []gaIndividual {
	pop := make([]gaIndividual, 0, popSize)
	mask := make([]bool, len(seed.seedWperm))
	for i := range mask {
		mask[i] = true
	}
	seedInd := gaIndividual{
		Wperm: append([]int(nil), seed.seedWperm...),
		Dperm: append([]int(nil), seed.seedDperm...),
		Mask:  mask,
	}
	seedInd = evaluateIndividual(seedInd, precompute)
	if seedInd.valid {
		pop = append(pop, seedInd)
	}

	tries := 0
	for len(pop) < popSize && tries < popSize*10 {
		tries++
		ind := gaIndividual{
			Wperm: append([]int(nil), seed.seedWperm...),
			Dperm: append([]int(nil), seed.seedDperm...),
		}
		ind.Mask = make([]bool, len(ind.Wperm))
		for i := range ind.Mask {
			ind.Mask[i] = rng.Float64() < 0.75
		}
		ensureAtLeastOneTrue(ind.Mask, rng)
		perturbIndividual(&ind, precompute, rng, 3)
		ind = evaluateIndividual(ind, precompute)
		if ind.valid {
			pop = append(pop, ind)
		}
	}
	if len(pop) == 0 {
		return []gaIndividual{seedInd}
	}
	return pop
}

func perturbIndividual(ind *gaIndividual, precompute gaPrecompute, rng *rand.Rand, swaps int) {
	for i := 0; i < swaps; i++ {
		if rng.Float64() < 0.5 {
			swapPositions(ind.Wperm, rng)
			continue
		}
		swapFeasibleDronePositionsMasked(ind.Dperm, ind.Mask, precompute, rng, 5)
	}
}

func evaluateIndividual(ind gaIndividual, precompute gaPrecompute) gaIndividual {
	if len(ind.Dperm) != len(precompute.tasks) || len(ind.Wperm) != len(precompute.tasks) || len(ind.Mask) != len(precompute.tasks) {
		ind.valid = false
		return ind
	}
	exec := 0
	doneH := 0
	nmax := 0
	nsum := 0
	esum := 0.0
	slackMin := math.Inf(1)
	for r := range precompute.tasks {
		if !ind.Mask[r] {
			continue
		}
		exec++
		dIdx := ind.Dperm[r]
		wIdx := ind.Wperm[r]
		dRow, okD := precompute.droneIndex[dIdx]
		wRow, okW := precompute.workerIndex[wIdx]
		if !okD || !okW || !precompute.feasible[dRow][r] {
			ind.valid = false
			return ind
		}
		nr := precompute.td[dRow][r]
		if precompute.tw[wRow][r] > nr {
			nr = precompute.tw[wRow][r]
		}
		if nr > nmax {
			nmax = nr
		}
		if nr <= precompute.horizon {
			doneH++
		}
		nsum += nr
		esum += precompute.eneed[dRow][r]
		slack := precompute.dpow[dRow] - precompute.eneed[dRow][r]
		if slack < slackMin {
			slackMin = slack
		}
	}
	if exec == 0 {
		ind.valid = false
		return ind
	}
	ind.fitness = gaFitness{
		Exec:     exec,
		DoneH:    doneH,
		SlackMin: slackMin,
		Nmax:     nmax,
		Navg:     float64(nsum) / float64(exec),
		Esum:     esum,
	}
	ind.valid = true
	return ind
}

func betterFitness(a, b gaFitness) bool {
	if a.DoneH != b.DoneH {
		return a.DoneH > b.DoneH
	}
	if a.Exec != b.Exec {
		return a.Exec > b.Exec
	}
	if a.SlackMin != b.SlackMin {
		return a.SlackMin > b.SlackMin
	}
	if a.Navg != b.Navg {
		return a.Navg < b.Navg
	}
	if a.Nmax != b.Nmax {
		return a.Nmax < b.Nmax
	}
	return a.Esum < b.Esum
}

func bestIndividual(pop []gaIndividual) gaIndividual {
	best := pop[0]
	for i := 1; i < len(pop); i++ {
		if betterFitness(pop[i].fitness, best.fitness) {
			best = pop[i]
		}
	}
	return best
}

func tournamentSelect(pop []gaIndividual, rng *rand.Rand, k int) gaIndividual {
	best := pop[rng.Intn(len(pop))]
	for i := 1; i < k; i++ {
		cand := pop[rng.Intn(len(pop))]
		if betterFitness(cand.fitness, best.fitness) {
			best = cand
		}
	}
	return best
}

func pmx(parentA []int, parentB []int, rng *rand.Rand) []int {
	n := len(parentA)
	child := make([]int, n)
	for i := range child {
		child[i] = -1
	}
	c1 := rng.Intn(n)
	c2 := rng.Intn(n)
	if c1 > c2 {
		c1, c2 = c2, c1
	}

	for i := c1; i <= c2; i++ {
		child[i] = parentA[i]
	}

	for i := c1; i <= c2; i++ {
		val := parentB[i]
		if contains(child, val) {
			continue
		}
		pos := i
		for {
			mapped := parentA[pos]
			pos = indexOf(parentB, mapped)
			if child[pos] == -1 {
				child[pos] = val
				break
			}
		}
	}

	for i := 0; i < n; i++ {
		if child[i] == -1 {
			child[i] = parentB[i]
		}
	}
	return child
}

func maskCrossover(a []bool, b []bool, rng *rand.Rand) []bool {
	n := len(a)
	if n == 0 {
		return nil
	}
	child := make([]bool, n)
	cut := rng.Intn(n)
	for i := 0; i < n; i++ {
		if i < cut {
			child[i] = a[i]
		} else {
			child[i] = b[i]
		}
	}
	return child
}

func repairDronePermMasked(dperm []int, mask []bool, seed []int, precompute gaPrecompute) bool {
	posByDrone := make(map[int]int)
	for i, d := range dperm {
		posByDrone[d] = i
	}

	for r := range precompute.tasks {
		if r >= len(mask) || !mask[r] {
			continue
		}
		if feasibleDroneAt(dperm[r], r, precompute) {
			continue
		}
		swapped := false
		for s := 0; s < len(dperm); s++ {
			if s == r || (s < len(mask) && !mask[s]) {
				continue
			}
			if feasibleDroneAt(dperm[s], r, precompute) && feasibleDroneAt(dperm[r], s, precompute) {
				dperm[r], dperm[s] = dperm[s], dperm[r]
				posByDrone[dperm[r]] = r
				posByDrone[dperm[s]] = s
				swapped = true
				break
			}
		}
		if swapped {
			continue
		}
		if r >= len(seed) {
			return false
		}
		target := seed[r]
		pos, ok := posByDrone[target]
		if !ok {
			return false
		}
		dperm[r], dperm[pos] = dperm[pos], dperm[r]
		posByDrone[dperm[r]] = r
		posByDrone[dperm[pos]] = pos
		if !feasibleDroneAt(dperm[r], r, precompute) {
			return false
		}
	}
	return true
}

func feasibleDroneAt(droneID int, taskPos int, precompute gaPrecompute) bool {
	dRow, ok := precompute.droneIndex[droneID]
	if !ok {
		return false
	}
	return precompute.feasible[dRow][taskPos]
}

func mutateIndividual(ind *gaIndividual, precompute gaPrecompute, rng *rand.Rand, pmW float64, pmD float64, pmM float64) {
	if rng.Float64() < pmW {
		swapPositions(ind.Wperm, rng)
	}
	if rng.Float64() < pmD {
		swapFeasibleDronePositionsMasked(ind.Dperm, ind.Mask, precompute, rng, 5)
	}
	if rng.Float64() < pmM {
		mutateMask(ind.Mask, rng, 0.2)
	}
	ensureAtLeastOneTrue(ind.Mask, rng)
}

func mutateMask(mask []bool, rng *rand.Rand, flipRate float64) {
	for i := range mask {
		if rng.Float64() < flipRate {
			mask[i] = !mask[i]
		}
	}
}

func swapPositions(perm []int, rng *rand.Rand) {
	if len(perm) < 2 {
		return
	}
	i := rng.Intn(len(perm))
	j := rng.Intn(len(perm))
	perm[i], perm[j] = perm[j], perm[i]
}

func ensureAtLeastOneTrue(mask []bool, rng *rand.Rand) {
	for _, v := range mask {
		if v {
			return
		}
	}
	if len(mask) == 0 {
		return
	}
	mask[rng.Intn(len(mask))] = true
}

func swapFeasibleDronePositionsMasked(perm []int, mask []bool, precompute gaPrecompute, rng *rand.Rand, tries int) {
	if len(perm) < 2 {
		return
	}
	for attempt := 0; attempt < tries; attempt++ {
		i := rng.Intn(len(perm))
		j := rng.Intn(len(perm))
		if i == j {
			continue
		}
		if i < len(mask) && !mask[i] {
			continue
		}
		if j < len(mask) && !mask[j] {
			continue
		}
		if feasibleDroneAt(perm[i], j, precompute) && feasibleDroneAt(perm[j], i, precompute) {
			perm[i], perm[j] = perm[j], perm[i]
			return
		}
	}
}

func buildDroneToTaskMasked(taskOrder []int, dperm []int, mask []bool) map[int]int {
	droneToTask := make(map[int]int)
	for r, taskIdx := range taskOrder {
		if r >= len(dperm) || r >= len(mask) {
			break
		}
		if !mask[r] {
			continue
		}
		droneToTask[dperm[r]] = taskIdx
	}
	return droneToTask
}

func buildTaskToWorkerMasked(taskOrder []int, wperm []int, mask []bool) map[int]int {
	taskToWorker := make(map[int]int)
	for r, taskIdx := range taskOrder {
		if r >= len(wperm) || r >= len(mask) {
			break
		}
		if !mask[r] {
			continue
		}
		taskToWorker[taskIdx] = wperm[r]
	}
	return taskToWorker
}

func contains(list []int, value int) bool {
	for _, v := range list {
		if v == value {
			return true
		}
	}
	return false
}

func indexOf(list []int, value int) int {
	for i, v := range list {
		if v == value {
			return i
		}
	}
	return -1
}

func indKey(ind gaIndividual) string {
	var b strings.Builder
	for _, v := range ind.Mask {
		if v {
			b.WriteString("1")
		} else {
			b.WriteString("0")
		}
	}
	b.WriteString("|")
	for _, v := range ind.Wperm {
		b.WriteString(fmt.Sprintf("w%d,", v))
	}
	b.WriteString("|")
	for _, v := range ind.Dperm {
		b.WriteString(fmt.Sprintf("d%d,", v))
	}
	return b.String()
}
