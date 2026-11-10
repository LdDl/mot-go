package mot

import "math"

// hungarianMatching minimizes assignment cost with one dummy column per track.
// Below-threshold edges cost more than leaving a track unmatched, so they cannot
// displace a valid match during optimization. Rows are tracks, columns detections.
func hungarianMatching(scores [][]float64, threshold float64) [][2]int {
	if len(scores) == 0 || len(scores[0]) == 0 {
		return nil
	}
	tracks, detections := len(scores), len(scores[0])
	columns := detections + tracks
	unmatchedCost := int64(SCALE_FACTOR) + 1
	forbiddenCost := unmatchedCost + 1
	cost := func(row, column int) int64 {
		if column >= detections {
			return unmatchedCost
		}
		score := scores[row][column]
		if !(score >= threshold) {
			return forbiddenCost
		}
		return int64((1 - score) * SCALE_FACTOR)
	}

	// Row/column potentials and predecessors for shortest augmenting paths.
	// Column zero is the temporary root; the real indices are one-based here.
	rowPotential := make([]int64, tracks+1)
	columnPotential := make([]int64, columns+1)
	assignedRow := make([]int, columns+1)
	predecessor := make([]int, columns+1)
	slack := make([]int64, columns+1)
	visited := make([]bool, columns+1)
	for row := 1; row <= tracks; row++ {
		assignedRow[0] = row
		clear(visited)
		for column := 1; column <= columns; column++ {
			slack[column] = math.MaxInt64
		}
		column := 0
		for {
			visited[column] = true
			currentRow := assignedRow[column]
			delta := int64(math.MaxInt64)
			nextColumn := 0
			for candidate := 1; candidate <= columns; candidate++ {
				if visited[candidate] {
					continue
				}
				reducedCost := cost(currentRow-1, candidate-1) - rowPotential[currentRow] - columnPotential[candidate]
				if reducedCost < slack[candidate] {
					slack[candidate] = reducedCost
					predecessor[candidate] = column
				}
				if slack[candidate] < delta {
					delta = slack[candidate]
					nextColumn = candidate
				}
			}
			for candidate := 0; candidate <= columns; candidate++ {
				if visited[candidate] {
					rowPotential[assignedRow[candidate]] += delta
					columnPotential[candidate] -= delta
				} else if candidate > 0 {
					slack[candidate] -= delta
				}
			}
			column = nextColumn
			if assignedRow[column] == 0 {
				break
			}
		}
		for column != 0 {
			previous := predecessor[column]
			assignedRow[column] = assignedRow[previous]
			column = previous
		}
	}
	matches := make([][2]int, 0, min(tracks, detections))
	for column := 1; column <= detections; column++ {
		row := assignedRow[column]
		if row != 0 && scores[row-1][column-1] >= threshold {
			matches = append(matches, [2]int{row - 1, column - 1})
		}
	}
	return matches
}
