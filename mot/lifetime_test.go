package mot

import (
	"math"
	"testing"

	"github.com/google/uuid"
)

// lifetimeTracker is the part of a tracker the lifetime tests care about, so the
// same scenarios run against every tracker type
type lifetimeTracker struct {
	name  string
	match func(frame []*SimpleBlob) error
	count func() int
	lost  func() float64
	setDt func(dt float64)
}

func newLifetimeTrackers(maxLostSeconds float64) []lifetimeTracker {
	iou := NewIoUTracker[*SimpleBlob](1000, 0.3)
	simple := NewNewSimpleTracker[*SimpleBlob](30.0, 1000)
	bt := NewByteTracker[*SimpleBlob](1000, 0.3, 0.5, 0.3, MatchingAlgorithmHungarian)
	if maxLostSeconds > 0 {
		iou.SetMaxLostSeconds(maxLostSeconds)
		simple.SetMaxLostSeconds(maxLostSeconds)
		bt.SetMaxLostSeconds(maxLostSeconds)
	}
	firstLost := func(objects map[uuid.UUID]*SimpleBlob) float64 {
		for _, o := range objects {
			return o.GetLostSeconds()
		}
		return math.NaN()
	}
	return []lifetimeTracker{
		{
			name:  "IoUTracker",
			match: func(frame []*SimpleBlob) error { return iou.MatchObjects(frame) },
			count: func() int { return len(iou.Objects) },
			lost:  func() float64 { return firstLost(iou.Objects) },
			setDt: iou.SetDt,
		},
		{
			name:  "SimpleTracker",
			match: func(frame []*SimpleBlob) error { return simple.MatchObjects(frame) },
			count: func() int { return len(simple.Objects) },
			lost:  func() float64 { return firstLost(simple.Objects) },
			setDt: simple.SetDt,
		},
		{
			name: "ByteTracker",
			match: func(frame []*SimpleBlob) error {
				conf := make([]float64, len(frame))
				for i := range conf {
					conf[i] = 0.9
				}
				return bt.MatchObjects(frame, conf)
			},
			count: func() int { return len(bt.Objects) },
			lost:  func() float64 { return firstLost(bt.Objects) },
			setDt: bt.SetDt,
		},
	}
}

func detection(dt float64) []*SimpleBlob {
	return []*SimpleBlob{NewSimpleBlobWithTime(NewRect(10, 10, 20, 20), dt)}
}

// With a time limit the track must go when its unmatched time exceeds the
// limit, no matter how generous the frame limit is
func TestExpiryBySeconds(t *testing.T) {
	const dt = 0.5
	for _, tr := range newLifetimeTrackers(1.0) {
		t.Run(tr.name, func(t *testing.T) {
			if err := tr.match(detection(dt)); err != nil {
				t.Fatal(err)
			}
			if tr.count() != 1 {
				t.Fatalf("expected 1 track after the first frame, got %d", tr.count())
			}
			// The object vanishes. Each empty frame adds dt to the lost time; the
			// track must survive exactly as long as that stays within the limit
			emptyFrames := 0
			for tr.count() > 0 {
				if lost := tr.lost(); lost > 1.0 {
					t.Fatalf("track kept while lost for %v s > 1 s", lost)
				}
				if err := tr.match(nil); err != nil {
					t.Fatal(err)
				}
				emptyFrames++
				if emptyFrames > 3 {
					t.Fatalf("track not expired by time after %d empty frames", emptyFrames)
				}
			}
			if emptyFrames < 2 {
				t.Fatalf("track expired too early, after %d empty frames", emptyFrames)
			}
		})
	}
}

// A frame without detections carries no dt, so the tracker must be told the
// real interval explicitly for the lost time to be counted right
func TestSetDtOnEmptyFrames(t *testing.T) {
	const nominalDt = 0.1
	withSetDt := newLifetimeTrackers(1.0)
	withoutSetDt := newLifetimeTrackers(1.0)
	for i := range withSetDt {
		t.Run(withSetDt[i].name, func(t *testing.T) {
			for _, tr := range []lifetimeTracker{withSetDt[i], withoutSetDt[i]} {
				if err := tr.match(detection(nominalDt)); err != nil {
					t.Fatal(err)
				}
				if tr.count() != 1 {
					t.Fatalf("expected 1 track after the first frame, got %d", tr.count())
				}
			}
			// Frames now arrive every 0.5 s instead of 0.1 s and the object is gone
			for f := 0; f < 4; f++ {
				withSetDt[i].setDt(0.5)
				if err := withSetDt[i].match(nil); err != nil {
					t.Fatal(err)
				}
				if err := withoutSetDt[i].match(nil); err != nil {
					t.Fatal(err)
				}
			}
			if withSetDt[i].count() != 0 {
				t.Errorf("told the real interval: 4 x 0.5 s > 1 s, track must be gone")
			}
			if withoutSetDt[i].count() != 1 {
				t.Errorf("still on the nominal interval: 4 x 0.1 s < 1 s, track must be kept")
			}
		})
	}
}

// Without a time limit the frame rule is untouched
func TestExpiryByFramesUnchanged(t *testing.T) {
	for _, tr := range newLifetimeTrackers(0) {
		t.Run(tr.name, func(t *testing.T) {
			if err := tr.match(detection(100.0)); err != nil {
				t.Fatal(err)
			}
			// Huge dt, but the frame rule does not care: two empty frames are within 1000
			for f := 0; f < 2; f++ {
				if err := tr.match(nil); err != nil {
					t.Fatal(err)
				}
			}
			if tr.count() != 1 {
				t.Errorf("expected the track to be kept by the frame rule, got %d tracks", tr.count())
			}
		})
	}
}

// Non-positive limits are ignored, the frame rule stays in effect
func TestNonPositiveLostSecondsIgnored(t *testing.T) {
	iou := NewIoUTracker[*SimpleBlob](10, 0.3)
	iou.SetMaxLostSeconds(0)
	iou.SetMaxLostSeconds(-1)
	if iou.GetMaxLostSeconds() != 0 {
		t.Errorf("expected frame-based expiry, got max lost seconds %v", iou.GetMaxLostSeconds())
	}
	iou.SetMaxLostSeconds(2)
	iou.SetMaxNoMatch(5)
	if iou.GetMaxLostSeconds() != 0 || iou.GetMaxNoMatch() != 5 {
		t.Errorf("SetMaxNoMatch must switch back to the frame rule")
	}
}

func TestLostSecondsFollowDt(t *testing.T) {
	blobs := map[string]interface {
		IncNoMatch()
		ResetNoMatch()
		SetDt(float64)
		GetNoMatchTimes() int
		GetLostSeconds() float64
	}{
		"SimpleBlob": NewSimpleBlobWithTime(NewRect(0, 0, 10, 10), 0.25),
		"BlobBBox":   NewBlobBBoxWithTime(NewRect(0, 0, 10, 10), 0.25),
	}
	for name, blob := range blobs {
		t.Run(name, func(t *testing.T) {
			if blob.GetLostSeconds() != 0 {
				t.Fatalf("fresh blob must have 0 lost seconds, got %v", blob.GetLostSeconds())
			}
			blob.IncNoMatch()
			blob.IncNoMatch()
			if blob.GetNoMatchTimes() != 2 || math.Abs(blob.GetLostSeconds()-0.5) > 1e-9 {
				t.Fatalf("expected 2 misses / 0.5 s, got %d / %v", blob.GetNoMatchTimes(), blob.GetLostSeconds())
			}
			// A new cycle time changes what the next miss is worth, not what was accumulated
			blob.SetDt(1.0)
			blob.IncNoMatch()
			if math.Abs(blob.GetLostSeconds()-1.5) > 1e-9 {
				t.Fatalf("expected 1.5 s, got %v", blob.GetLostSeconds())
			}
			blob.ResetNoMatch()
			if blob.GetNoMatchTimes() != 0 || blob.GetLostSeconds() != 0 {
				t.Fatalf("reset must clear both counters")
			}
		})
	}
}
