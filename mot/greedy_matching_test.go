package mot

import (
	"reflect"
	"slices"
	"testing"
)

func TestGreedyMatchesGlobalReference(t *testing.T) {
	for _, tracks := range []int{0, 1, 2, 7, 31} {
		for _, detections := range []int{0, 1, 3, 11, 32} {
			for order := 0; order < 3; order++ {
				for _, divisor := range []int{1, 3} {
					rows := make([][]matchCandidate, detections)
					pairs := make([]pendingMatch, 0)
					for d := 0; d < detections; d++ {
						for tr := 0; tr < tracks; tr++ {
							cost := d*tracks + tr
							if order == 1 {
								cost = tracks*detections - 1 - cost
							}
							if order == 2 {
								cost = ((tr+d)%tracks)*detections + d
							}
							if cost%divisor != 0 {
								continue
							}
							rows[d] = append(rows[d], matchCandidate{tr, float64(cost)})
							pairs = append(pairs, pendingMatch{d, tr, float64(cost)})
						}
					}
					slices.SortFunc(pairs, func(a, b pendingMatch) int { return int(a.cost - b.cost) })
					usedTracks, usedDetections := make([]bool, tracks), make([]bool, detections)
					want := make([][2]int, 0)
					for _, p := range pairs {
						if !usedTracks[p.track] && !usedDetections[p.detection] {
							usedTracks[p.track], usedDetections[p.detection] = true, true
							want = append(want, [2]int{p.detection, p.track})
						}
					}
					got := greedyMatching(rows, tracks)
					if !reflect.DeepEqual(got, want) {
						t.Fatalf("tracks=%d detections=%d order=%d divisor=%d: %v != %v", tracks, detections, order, divisor, got, want)
					}
				}
			}
		}
	}
}

func TestGreedyEqualCostAlternatives(t *testing.T) {
	rows := [][]matchCandidate{{{0, 0}}, {{0, 1}, {1, 2}, {2, 2}}, nil}
	got := greedyMatching(rows, 3)
	if !reflect.DeepEqual(got, [][2]int{{0, 0}, {1, 1}}) {
		t.Fatalf("equal costs lost original track order: %v", got)
	}
}

type competitionCase struct {
	name          string
	tracks        []float64
	detections    []float64
	expectedTrack []int
}

func permutations(values []int) [][]int {
	if len(values) == 0 {
		return [][]int{{}}
	}
	result := make([][]int, 0)
	for i, value := range values {
		rest := append([]int{}, values[:i]...)
		rest = append(rest, values[i+1:]...)
		for _, tail := range permutations(rest) {
			result = append(result, append([]int{value}, tail...))
		}
	}
	return result
}

func checkCompetition[B observableBlob[B]](t *testing.T, makeBlob func(Rectangle, float64) B) {
	cases := []competitionCase{
		{"Fallback", []float64{100, 130}, []float64{101, 110}, []int{0, 1}},
		{"Requeue", []float64{100, 130, 160}, []float64{101, 110, 118}, []int{0, 2, 1}},
		{"OnlyOneTrack", []float64{100}, []float64{101, 110}, []int{0, -1}},
		{"NoSuitableAlternative", []float64{100, 1000}, []float64{101, 110}, []int{0, -1}},
	}
	for _, tc := range cases {
		indices := make([]int, len(tc.detections))
		for i := range indices {
			indices[i] = i
		}
		for _, order := range permutations(indices) {
			for _, tr := range regressionTrackers[B]()[:2] {
				t.Run(tc.name+"/"+tr.name, func(t *testing.T) {
					initial := make([]B, len(tc.tracks))
					for i, x := range tc.tracks {
						initial[i] = makeBlob(NewRect(x, 100, 100, 100), 0.04)
					}
					if err := tr.match(initial, nil); err != nil {
						t.Fatal(err)
					}
					detections := make([]B, len(order))
					for i, index := range order {
						detections[i] = makeBlob(NewRect(tc.detections[index], 100, 100, 100), 0.04)
					}
					if err := tr.match(detections, nil); err != nil {
						t.Fatal(err)
					}
					newTracks := 0
					for i, index := range order {
						target := tc.expectedTrack[index]
						if target < 0 {
							newTracks++
							for _, old := range initial {
								if detections[i].GetID() == old.GetID() {
									t.Fatal("unmatched detection stole a track")
								}
							}
						} else if detections[i].GetID() != initial[target].GetID() {
							t.Fatalf("detection %d assigned incorrectly", index)
						}
					}
					if len(tr.objects) != len(initial)+newTracks {
						t.Fatal("unexpected extra tracks")
					}
					for _, old := range initial {
						if len(old.GetTrack()) > 2 {
							t.Fatal("track updated more than once")
						}
					}
				})
			}
		}
	}
}

func TestCompetingDetections(t *testing.T) {
	t.Run("SimpleBlob", func(t *testing.T) { checkCompetition(t, NewSimpleBlobWithTime) })
	t.Run("BlobBBox", func(t *testing.T) { checkCompetition(t, NewBlobBBoxWithTime) })
}
