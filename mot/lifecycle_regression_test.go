package mot

import (
	"fmt"
	"reflect"
	"testing"

	"github.com/google/uuid"
)

type observableBlob[B any] interface {
	Blob[B]
	Exists() bool
}

type trackerHarness[B Blob[B]] struct {
	name       string
	objects    map[uuid.UUID]B
	match      func([]B, []float64) error
	setFrames  func(int)
	setSeconds func(float64)
	setDt      func(float64)
}

func regressionTrackers[B Blob[B]]() []trackerHarness[B] {
	simple := NewSimpleTrackerDefault[B]()
	iou := NewDefaultIoUTracker[B]()
	result := []trackerHarness[B]{
		{"Simple", simple.Objects, func(d []B, _ []float64) error { return simple.MatchObjects(d) }, simple.SetMaxNoMatch, simple.SetMaxLostSeconds, simple.SetDt},
		{"IoU", iou.Objects, func(d []B, _ []float64) error { return iou.MatchObjects(d) }, iou.SetMaxNoMatch, iou.SetMaxLostSeconds, iou.SetDt},
	}
	for _, algorithm := range []MatchingAlgorithm{MatchingAlgorithmHungarian, MatchingAlgorithmGreedy} {
		bt := NewByteTracker[B](75, 0.3, 0.5, 0.3, algorithm)
		name := "ByteHungarian"
		if algorithm == MatchingAlgorithmGreedy {
			name = "ByteGreedy"
		}
		result = append(result, trackerHarness[B]{name, bt.Objects, func(d []B, c []float64) error {
			if c == nil {
				c = make([]float64, len(d))
				for i := range c {
					c[i] = 0.9
				}
			}
			return bt.MatchObjects(d, c)
		}, bt.SetMaxDisappeared, bt.SetMaxLostSeconds, bt.SetDt})
	}
	return result
}

func checkLifecycle[B observableBlob[B]](t *testing.T, makeBlob func(Rectangle, float64) B) {
	for _, tr := range regressionTrackers[B]() {
		t.Run(tr.name, func(t *testing.T) {
			a := makeBlob(NewRect(100, 100, 100, 100), 0.125)
			b := makeBlob(NewRect(1000, 100, 100, 100), 0.125)
			aID, bID := a.GetID(), b.GetID()
			before := a.GetPredictedBBox()
			if err := tr.match([]B{a, b}, nil); err != nil {
				t.Fatal(err)
			}
			if a.GetPredictedBBox() != before {
				t.Fatal("creation advanced the filter")
			}
			for _, blob := range tr.objects {
				if !blob.Exists() || blob.GetNoMatchTimes() != 0 || blob.GetLostSeconds() != 0 {
					t.Fatal("new track is inactive or missed")
				}
			}
			nextA := makeBlob(NewRect(101, 100, 100, 100), 0.25)
			c := makeBlob(NewRect(2000, 100, 100, 100), 0.25)
			cID := c.GetID()
			if err := tr.match([]B{nextA, c}, nil); err != nil {
				t.Fatal(err)
			}
			if len(tr.objects) != 3 || nextA.GetID() != aID {
				t.Fatal("mixed frame changed identities")
			}
			if b.Exists() || b.GetNoMatchTimes() != 1 || b.GetLostSeconds() != 0.25 {
				t.Fatal("miss must use incoming dt")
			}
			for _, id := range []uuid.UUID{aID, cID} {
				blob := tr.objects[id]
				if !blob.Exists() || blob.GetNoMatchTimes() != 0 || blob.GetLostSeconds() != 0 {
					t.Fatal("matched/new track counted as missed")
				}
			}
			if err := tr.match(nil, nil); err != nil {
				t.Fatal(err)
			}
			if len(tr.objects) != 3 {
				t.Fatal("missed tracks expired too soon")
			}
			for _, blob := range tr.objects {
				if blob.Exists() {
					t.Fatal("empty frame left an active track")
				}
			}
			if b.GetNoMatchTimes() != 2 || b.GetLostSeconds() != 0.5 {
				t.Fatal("empty frame lost the previous dt")
			}
			for i := 0; i < 3; i++ {
				recovered := makeBlob(NewRect(1000, 100, 100, 100), 0.125)
				if err := tr.match([]B{recovered}, nil); err != nil {
					t.Fatal(err)
				}
				if recovered.GetID() != bID || !b.Exists() || b.GetNoMatchTimes() != 0 || b.GetLostSeconds() != 0 {
					t.Fatal("recovery must retain ID and reset misses")
				}
			}
		})
	}
	for _, tr := range regressionTrackers[B]() {
		t.Run(tr.name+"/ZeroFrameLimit", func(t *testing.T) {
			tr.setFrames(0)
			var id uuid.UUID
			for frame := 0; frame < 3; frame++ {
				blob := makeBlob(NewRect(100, 100, 100, 100), 0.125)
				if frame == 0 {
					id = blob.GetID()
				}
				if err := tr.match([]B{blob}, nil); err != nil {
					t.Fatal(err)
				}
				if len(tr.objects) != 1 || blob.GetID() != id {
					t.Fatal("zero limit removed a detected track")
				}
			}
			if err := tr.match(nil, nil); err != nil {
				t.Fatal(err)
			}
			if len(tr.objects) != 0 {
				t.Fatal("zero frame limit kept a missed track")
			}
		})
	}
	for _, tr := range regressionTrackers[B]() {
		t.Run(tr.name+"/ShortTimeLimit", func(t *testing.T) {
			tr.setSeconds(0.01)
			blob := makeBlob(NewRect(100, 100, 100, 100), 0.5)
			if err := tr.match([]B{blob}, nil); err != nil {
				t.Fatal(err)
			}
			if len(tr.objects) != 1 || blob.GetLostSeconds() != 0 {
				t.Fatal("creation consumed the time limit")
			}
			if err := tr.match(nil, nil); err != nil {
				t.Fatal(err)
			}
			if len(tr.objects) != 0 {
				t.Fatal("miss should exceed the time limit")
			}
		})
	}
}

func TestTrackLifecycleRegression(t *testing.T) {
	t.Run("SimpleBlob", func(t *testing.T) { checkLifecycle(t, NewSimpleBlobWithTime) })
	t.Run("BlobBBox", func(t *testing.T) { checkLifecycle(t, NewBlobBBoxWithTime) })
}

func checkByteLimits[B observableBlob[B]](t *testing.T, makeBlob func(Rectangle, float64) B) {
	for _, algorithm := range []MatchingAlgorithm{MatchingAlgorithmHungarian, MatchingAlgorithmGreedy} {
		for _, limit := range []int{0, 1, 2} {
			for _, setter := range []bool{false, true} {
				t.Run(fmt.Sprintf("Algorithm%d/Limit%d/Setter%v", algorithm, limit, setter), func(t *testing.T) {
					bt := NewByteTracker[B](limit, 0.3, 0.5, 0.3, algorithm)
					if setter {
						bt.SetMaxLostSeconds(10)
						bt.SetMaxDisappeared(limit)
						if bt.GetMaxLostSeconds() != 0 {
							t.Fatal("frame setter must disable time expiry")
						}
					}
					if bt.GetMaxDisappeared() != limit {
						t.Fatal("configured limit changed")
					}
					first := makeBlob(NewRect(100, 100, 100, 100), 0.125)
					id := first.GetID()
					if err := bt.MatchObjects([]B{first}, []float64{0.9}); err != nil {
						t.Fatal(err)
					}
					for _, confidence := range []float64{0.9, 0.4} {
						next := makeBlob(NewRect(101, 100, 100, 100), 0.125)
						if err := bt.MatchObjects([]B{next}, []float64{confidence}); err != nil {
							t.Fatal(err)
						}
						if next.GetID() != id || len(bt.GetActiveTracks()) != 1 {
							t.Fatal("detected track lost at frame limit")
						}
					}
					if err := bt.MatchObjects([]B{first}, nil); err == nil {
						t.Fatal("expected invalid confidence length error")
					}
					if !first.Exists() || first.GetNoMatchTimes() != 0 {
						t.Fatal("invalid input mutated track activity")
					}
					for missed := 1; missed <= max(1, limit); missed++ {
						if err := bt.MatchObjects(nil, nil); err != nil {
							t.Fatal(err)
						}
						want := 1
						if missed == max(1, limit) {
							want = 0
						}
						if len(bt.Objects) != want || len(bt.GetActiveTracks()) != want {
							t.Fatalf("miss %d: incorrect expiry", missed)
						}
					}
				})
			}
		}
		t.Run(fmt.Sprintf("Algorithm%d/TimeOverridesZero", algorithm), func(t *testing.T) {
			bt := NewByteTracker[B](0, 0.3, 0.5, 0.3, algorithm)
			bt.SetMaxLostSeconds(0.25)
			blob := makeBlob(NewRect(100, 100, 100, 100), 0.125)
			if err := bt.MatchObjects([]B{blob}, []float64{0.9}); err != nil {
				t.Fatal(err)
			}
			for missed := 1; missed <= 3; missed++ {
				if err := bt.MatchObjects(nil, nil); err != nil {
					t.Fatal(err)
				}
				want := 1
				if missed == 3 {
					want = 0
				}
				if len(bt.GetActiveTracks()) != want {
					t.Fatal("time limit did not override frame limit")
				}
			}
		})
		t.Run(fmt.Sprintf("Algorithm%d/LowConfidenceRecovery", algorithm), func(t *testing.T) {
			bt := NewByteTracker[B](5, 0.3, 0.5, 0.3, algorithm)
			blob := makeBlob(NewRect(100, 100, 100, 100), 0.125)
			id := blob.GetID()
			if err := bt.MatchObjects([]B{blob}, []float64{0.9}); err != nil {
				t.Fatal(err)
			}
			if err := bt.MatchObjects(nil, nil); err != nil {
				t.Fatal(err)
			}
			if blob.Exists() || len(bt.GetActiveTracks()) != 1 {
				t.Fatal("missed track should remain recoverable")
			}
			next := makeBlob(NewRect(101, 100, 100, 100), 0.125)
			if err := bt.MatchObjects([]B{next}, []float64{0.4}); err != nil {
				t.Fatal(err)
			}
			if next.GetID() != id || !blob.Exists() || blob.GetNoMatchTimes() != 0 || blob.GetLostSeconds() != 0 {
				t.Fatal("low confidence recovery failed")
			}
		})
	}
}

func TestByteFrameLimits(t *testing.T) {
	t.Run("SimpleBlob", func(t *testing.T) { checkByteLimits(t, NewSimpleBlobWithTime) })
	t.Run("BlobBBox", func(t *testing.T) { checkByteLimits(t, NewBlobBBoxWithTime) })
}

func checkHistory[B Blob[B]](t *testing.T, makeBlob func(Rectangle, float64) B) {
	blob := makeBlob(NewRect(100, 100, 10, 10), 0.04)
	var all []Point
	all = append(all, blob.GetCenter())
	update := func(x int) {
		t.Helper()
		blob.PredictNextPosition()
		if err := blob.Update(makeBlob(NewRect(float64(x), 100, 10, 10), 0.04)); err != nil {
			t.Fatal(err)
		}
		all = append(all, blob.GetCenter())
	}
	for i := 101; i < 110; i++ {
		update(i)
	}
	blob.SetMaxTrackLen(3)
	if !reflect.DeepEqual(blob.GetTrack(), all[len(all)-3:]) {
		t.Fatal("shrinking must immediately keep newest points")
	}
	storage, capacity := &blob.GetTrack()[0], cap(blob.GetTrack())
	for i := 110; i < 120; i++ {
		update(i)
		if !reflect.DeepEqual(blob.GetTrack(), all[len(all)-3:]) {
			t.Fatal("history contains stale points")
		}
		if &blob.GetTrack()[0] != storage || cap(blob.GetTrack()) != capacity {
			t.Fatal("bounded history replaced its buffer")
		}
	}
	blob.SetMaxTrackLen(0)
	if len(blob.GetTrack()) != 0 {
		t.Fatal("zero must clear history immediately")
	}
	old := blob.GetCenter()
	update(500)
	if len(blob.GetTrack()) != 0 || blob.GetCenter() == old {
		t.Fatal("disabled history must not disable tracking")
	}
	blob.SetMaxTrackLen(2)
	if len(blob.GetTrack()) != 0 {
		t.Fatal("reenabling resurrected old points")
	}
	update(501)
	update(502)
	blob.SetMaxTrackLen(5)
	if !reflect.DeepEqual(blob.GetTrack(), all[len(all)-2:]) {
		t.Fatal("growing changed history")
	}
	update(503)
	blob.SetMaxTrackLen(1)
	if !reflect.DeepEqual(blob.GetTrack(), all[len(all)-1:]) {
		t.Fatal("limit one must keep newest point")
	}
	blob.SetMaxTrackLen(-1)
	update(504)
	if blob.GetMaxTrackLen() != 0 || len(blob.GetTrack()) != 0 {
		t.Fatal("negative Go limit must safely disable history")
	}
}

func TestHistoryLimits(t *testing.T) {
	t.Run("SimpleBlob", func(t *testing.T) { checkHistory(t, NewSimpleBlobWithTime) })
	t.Run("BlobBBox", func(t *testing.T) { checkHistory(t, NewBlobBBoxWithTime) })
}
