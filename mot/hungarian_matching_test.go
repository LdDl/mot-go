package mot

import (
	"math"
	"testing"
)

// Exhaustive enumeration is independent of the Hungarian implementation and
// checks the minimum total cost, including the choice to leave tracks unmatched.
func exhaustiveAssignmentCost(scores [][]float64, threshold float64) int64 {
	used := make([]bool, len(scores[0]))
	var search func(int) int64
	search = func(row int) int64 {
		if row == len(scores) {
			return 0
		}
		best := int64(SCALE_FACTOR) + 1 + search(row+1)
		for column, score := range scores[row] {
			if used[column] || score < threshold {
				continue
			}
			used[column] = true
			value := int64((1-score)*SCALE_FACTOR) + search(row+1)
			used[column] = false
			if value < best {
				best = value
			}
		}
		return best
	}
	return search(0)
}

func TestHungarianMatchesExhaustiveOptimum(t *testing.T) {
	levels := []float64{0, 0.25, 0.5, 0.75, 1}
	for rows := 1; rows <= 4; rows++ {
		for columns := 1; columns <= 4; columns++ {
			for pattern := 0; pattern < 20; pattern++ {
				scores := make([][]float64, rows)
				for row := range scores {
					scores[row] = make([]float64, columns)
					for column := range scores[row] {
						// Rotate rows and columns through a small explicit set of scores.
						index := (row*(pattern+1) + column*(pattern/len(levels)+1) + row*column + pattern) % len(levels)
						scores[row][column] = levels[index]
					}
				}
				for _, threshold := range []float64{0, 0.5, 1} {
					matches := hungarianMatching(scores, threshold)
					total := int64(rows-len(matches)) * (int64(SCALE_FACTOR) + 1)
					usedRows, usedColumns := make([]bool, rows), make([]bool, columns)
					for _, pair := range matches {
						row, column := pair[0], pair[1]
						if usedRows[row] || usedColumns[column] || scores[row][column] < threshold {
							t.Fatalf("invalid assignment %v for %v", matches, scores)
						}
						usedRows[row], usedColumns[column] = true, true
						total += int64((1 - scores[row][column]) * SCALE_FACTOR)
					}
					want := exhaustiveAssignmentCost(scores, threshold)
					if total != want {
						t.Fatalf("scores=%v threshold=%g: cost %d, optimum %d", scores, threshold, total, want)
					}
				}
			}
		}
	}
}

func TestHungarianRejectsNaN(t *testing.T) {
	matches := hungarianMatching([][]float64{{math.NaN(), 0.8}}, 0.5)
	if len(matches) != 1 || matches[0] != [2]int{0, 1} {
		t.Fatalf("invalid score entered matching: %v", matches)
	}
}

func checkByteAssociationStages[B observableBlob[B]](t *testing.T, makeBlob func(Rectangle, float64) B) {
	for _, algorithm := range []MatchingAlgorithm{MatchingAlgorithmHungarian, MatchingAlgorithmGreedy} {
		for _, confidence := range []float64{0.9, 0.4} {
			bt := NewByteTracker[B](5, 0.5, 0.5, 0.3, algorithm)
			first := []B{makeBlob(NewRect(100, 100, 100, 100), 0.04), makeBlob(NewRect(166, 100, 100, 100), 0.04)}
			if err := bt.MatchObjects(first, []float64{0.9, 0.9}); err != nil {
				t.Fatal(err)
			}
			frame := []B{makeBlob(NewRect(132, 100, 100, 100), 0.04), makeBlob(NewRect(66, 100, 100, 100), 0.04)}
			unmatchedID := frame[1].GetID()
			if err := bt.MatchObjects(frame, []float64{confidence, confidence}); err != nil {
				t.Fatal(err)
			}
			if frame[0].GetID() != first[0].GetID() || frame[1].GetID() != unmatchedID {
				t.Fatal("threshold displaced a valid association")
			}
			if first[0].GetNoMatchTimes() != 0 || first[1].GetNoMatchTimes() != 1 {
				t.Fatal("invalid counters after gated association")
			}
			want := 2
			if confidence >= 0.5 {
				want = 3
			}
			if len(bt.Objects) != want {
				t.Fatal("only unmatched high confidence detections may start tracks")
			}
		}
		bt := NewByteTracker[B](5, 0.3, 0.5, 0.3, algorithm)
		initial := makeBlob(NewRect(100, 100, 100, 100), 0.04)
		if err := bt.MatchObjects([]B{initial}, []float64{0.9}); err != nil {
			t.Fatal(err)
		}
		high := makeBlob(NewRect(110, 100, 100, 100), 0.04)
		low := makeBlob(NewRect(100, 100, 100, 100), 0.04)
		if err := bt.MatchObjects([]B{low, high}, []float64{0.4, 0.9}); err != nil {
			t.Fatal(err)
		}
		if high.GetID() != initial.GetID() || low.GetID() == initial.GetID() || len(bt.Objects) != 1 {
			t.Fatal("low confidence detection stole a high confidence match")
		}
	}
}

func TestByteAssociationStages(t *testing.T) {
	t.Run("SimpleBlob", func(t *testing.T) { checkByteAssociationStages(t, NewSimpleBlobWithTime) })
	t.Run("BlobBBox", func(t *testing.T) { checkByteAssociationStages(t, NewBlobBBoxWithTime) })
}
