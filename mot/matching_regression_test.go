package mot

import (
	"reflect"
	"sort"
	"testing"
)

type assignmentTestCase struct {
	name      string
	scores    [][]float64
	threshold float64
	want      [][2]int
}

func TestByteAssignment(t *testing.T) {
	cases := []assignmentTestCase{
		{"GatingSquare", [][]float64{{0.51, 0.49}, {0.49, 0}}, 0.5, [][2]int{{0, 0}}},
		{"GatingWide", [][]float64{{0.51, 0.49, 0}, {0.49, 0, 0}}, 0.5, [][2]int{{0, 0}}},
		{"GatingTall", [][]float64{{0.51, 0.49}, {0.49, 0}, {0, 0}}, 0.5, [][2]int{{0, 0}}},
		{"BestTrack", [][]float64{{0.55}, {0.95}}, 0.5, [][2]int{{1, 0}}},
		{"NoMatch", [][]float64{{0.49, 0.49}, {0.49, 0.49}}, 0.5, nil},
		{"ZeroThreshold", [][]float64{{0}}, 0, [][2]int{{0, 0}}},
		{"ExactThreshold", [][]float64{{0.5}}, 0.5, [][2]int{{0, 0}}},
		{"PerfectMatch", [][]float64{{1}}, 1, [][2]int{{0, 0}}},
		{"NoTracks", nil, 0.5, nil},
		{"NoDetections", [][]float64{{}, {}}, 0.5, nil},
	}
	for _, algorithm := range []MatchingAlgorithm{MatchingAlgorithmHungarian, MatchingAlgorithmGreedy} {
		name := "Hungarian"
		if algorithm == MatchingAlgorithmGreedy {
			name = "Greedy"
		}
		for _, tc := range cases {
			t.Run(name+"/"+tc.name, func(t *testing.T) {
				tracker := NewByteTracker[*SimpleBlob](5, tc.threshold, 0.5, 0.3, algorithm)
				tracks := make([]bboxPair, len(tc.scores))
				columns := 0
				if len(tc.scores) > 0 {
					columns = len(tc.scores[0])
				}
				got := tracker.performMatching(tc.scores, tracks, make([]int, columns))
				sort.Slice(got, func(i, j int) bool { return got[i][0] < got[j][0] })
				if len(got) != len(tc.want) || len(got) > 0 && !reflect.DeepEqual(got, tc.want) {
					t.Fatalf("assignments %v, want %v", got, tc.want)
				}
			})
		}
	}
}

func TestHungarianGlobalOptimum(t *testing.T) {
	tracker := NewByteTracker[*SimpleBlob](5, 0.5, 0.5, 0.3, MatchingAlgorithmHungarian)
	scores := [][]float64{{0.9, 0.8}, {0.85, 0.6}}
	got := tracker.performMatching(scores, make([]bboxPair, 2), make([]int, 2))
	sort.Slice(got, func(i, j int) bool { return got[i][0] < got[j][0] })
	if !reflect.DeepEqual(got, [][2]int{{0, 1}, {1, 0}}) {
		t.Fatalf("not globally optimal: %v", got)
	}
}
