package mot

import "slices"

type matchCandidate struct {
	track int
	cost  float64
}

type pendingMatch struct {
	detection int
	track     int
	cost      float64
}

type matchHeap []pendingMatch

func (h *matchHeap) push(item pendingMatch) {
	*h = append(*h, item)
	for child := len(*h) - 1; child > 0; {
		parent := (child - 1) / 2
		if (*h)[parent].cost <= (*h)[child].cost {
			break
		}
		(*h)[parent], (*h)[child] = (*h)[child], (*h)[parent]
		child = parent
	}
}

func (h *matchHeap) pop() pendingMatch {
	result := (*h)[0]
	last := len(*h) - 1
	(*h)[0] = (*h)[last]
	*h = (*h)[:last]
	for parent := 0; 2*parent+1 < len(*h); {
		child := 2*parent + 1
		if child+1 < len(*h) && (*h)[child+1].cost < (*h)[child].cost {
			child++
		}
		if (*h)[parent].cost <= (*h)[child].cost {
			break
		}
		(*h)[parent], (*h)[child] = (*h)[child], (*h)[parent]
		parent = child
	}
	return result
}

// greedyMatching consumes acceptable candidates per detection and returns
// (detection, track) pairs in increasing cost order, reserving each endpoint once.
func greedyMatching(candidates [][]matchCandidate, numTracks int) [][2]int {
	pending := make(matchHeap, 0, len(candidates))
	for detection, row := range candidates {
		if len(row) == 0 {
			continue
		}
		best := 0
		for i := 1; i < len(row); i++ {
			if row[i].cost < row[best].cost {
				best = i
			}
		}
		candidate := row[best]
		row[best] = row[len(row)-1]
		candidates[detection] = row[:len(row)-1]
		pending.push(pendingMatch{detection, candidate.track, candidate.cost})
	}
	reserved := make([]bool, numTracks)
	sorted := make([]bool, len(candidates))
	matches := make([][2]int, 0, min(numTracks, len(candidates)))
	for len(pending) > 0 {
		candidate := pending.pop()
		if !reserved[candidate.track] {
			reserved[candidate.track] = true
			matches = append(matches, [2]int{candidate.detection, candidate.track})
			continue
		}
		row := candidates[candidate.detection]
		if !sorted[candidate.detection] {
			// Sort alternatives only after a conflict, with the best at the end.
			slices.SortFunc(row, func(a, b matchCandidate) int {
				if a.cost < b.cost {
					return 1
				}
				if a.cost > b.cost {
					return -1
				}
				return b.track - a.track
			})
			sorted[candidate.detection] = true
		}
		for len(row) > 0 {
			next := row[len(row)-1]
			row = row[:len(row)-1]
			if !reserved[next.track] {
				// The fallback competes with other detections at its own cost.
				pending.push(pendingMatch{candidate.detection, next.track, next.cost})
				break
			}
		}
		candidates[candidate.detection] = row
	}
	return matches
}
