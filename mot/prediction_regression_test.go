package mot

import (
	"math"
	"testing"

	"github.com/google/uuid"
)

func nearRect(a, b Rectangle) bool {
	return math.Abs(a.X-b.X) < 1e-9 && math.Abs(a.Y-b.Y) < 1e-9 && math.Abs(a.Width-b.Width) < 1e-9 && math.Abs(a.Height-b.Height) < 1e-9
}

func checkPrediction[B Blob[B]](t *testing.T, makeBlob func(Rectangle, float64) B, filterBBox func(B) Rectangle, uncertainty func(B) float64) {
	blob := makeBlob(NewRect(100, 100, 30, 40), 0.04)
	for step, dt := range []float64{0.04, 0.2, 0.125, 0.04} {
		blob.SetDt(dt)
		stateBefore, uncertaintyBefore := filterBBox(blob), uncertainty(blob)
		prediction := blob.GetPredictedBBox()
		detection := makeBlob(NewRect(120+float64(step), 110, 35, 42), dt)
		expectedDistance := euclideanDistance(detection.GetCenter(), Point{X: prediction.X + prediction.Width/2, Y: prediction.Y + prediction.Height/2})
		for i := 0; i < 3; i++ {
			if blob.GetPredictedBBox() != prediction || math.Abs(detection.DistanceToPredicted(blob)-expectedDistance) > 1e-9 {
				t.Fatal("inconsistent readonly prediction")
			}
		}
		if filterBBox(blob) != stateBefore || uncertainty(blob) != uncertaintyBefore {
			t.Fatal("scoring advanced filter state or covariance")
		}
		blob.PredictNextPosition()
		if !nearRect(filterBBox(blob), prediction) {
			t.Fatalf("readonly prediction %v differs from actual %v", prediction, filterBBox(blob))
		}
		if err := blob.Update(detection); err != nil {
			t.Fatal(err)
		}
	}
}

func TestReadonlyPrediction(t *testing.T) {
	t.Run("SimpleBlob", func(t *testing.T) {
		checkPrediction(t, NewSimpleBlobWithTime, func(blob *SimpleBlob) Rectangle {
			x, y := blob.tracker.GetState()
			box := blob.GetBBox()
			return NewRect(x-box.Width/2, y-box.Height/2, box.Width, box.Height)
		}, func(blob *SimpleBlob) float64 { return blob.tracker.P.At(0, 0) + blob.tracker.P.At(1, 1) })
	})
	t.Run("BlobBBox", func(t *testing.T) {
		checkPrediction(t, NewBlobBBoxWithTime, func(blob *BlobBBox) Rectangle {
			x, y, w, h := blob.tracker.GetState()
			return NewRect(x-w/2, y-h/2, w, h)
		}, func(blob *BlobBBox) float64 { return blob.tracker.GetPositionUncertainty() })
	})
}

type customMetricBlob struct {
	*SimpleBlob
	preferred       uuid.UUID
	distanceCalls   int
	predictionCalls int
}

func (b *customMetricBlob) DistanceTo(other *customMetricBlob) float64 {
	b.distanceCalls++
	return 1000
}

func (b *customMetricBlob) DistanceToPredicted(other *customMetricBlob) float64 {
	b.predictionCalls++
	if other.GetID() == b.preferred {
		return 1
	}
	return 1000
}

func (b *customMetricBlob) Update(other *customMetricBlob) error {
	return b.SimpleBlob.Update(other.SimpleBlob)
}

func TestSimplePreservesCustomMetrics(t *testing.T) {
	tr := NewSimpleTrackerDefault[*customMetricBlob]()
	a := &customMetricBlob{SimpleBlob: NewSimpleBlob(NewRect(0, 0, 10, 10))}
	b := &customMetricBlob{SimpleBlob: NewSimpleBlob(NewRect(1000, 0, 10, 10))}
	if err := tr.MatchObjects([]*customMetricBlob{a, b}); err != nil {
		t.Fatal(err)
	}
	detection := &customMetricBlob{SimpleBlob: NewSimpleBlob(NewRect(0, 0, 10, 10)), preferred: b.GetID()}
	if err := tr.MatchObjects([]*customMetricBlob{detection}); err != nil {
		t.Fatal(err)
	}
	if detection.GetID() != b.GetID() || detection.distanceCalls != 2 || detection.predictionCalls != 2 {
		t.Fatal("tracker bypassed or repeated custom metrics")
	}
}

func checkIoUDefaults[B Blob[B]](t *testing.T, makeBlob func(Rectangle, float64) B) {
	for _, threshold := range []float64{0.3, 0} {
		tracker := NewIoUTracker[B](75, threshold)
		if threshold == 0.3 {
			tracker = NewDefaultIoUTracker[B]()
		}
		first := makeBlob(NewRect(100, 100, 2, 2), 0.04)
		id := first.GetID()
		if err := tracker.MatchObjects([]B{first}); err != nil {
			t.Fatal(err)
		}
		nearby := makeBlob(NewRect(110, 100, 2, 2), 0.04)
		if err := tracker.MatchObjects([]B{nearby}); err != nil {
			t.Fatal(err)
		}
		if nearby.GetID() != id {
			t.Fatal("distance fallback rejected a nearby non-overlapping box")
		}
		far := makeBlob(NewRect(10000, 100, 2, 2), 0.04)
		if err := tracker.MatchObjects([]B{far}); err != nil {
			t.Fatal(err)
		}
		if (far.GetID() == id) != (threshold == 0) {
			t.Fatal("default must reject far detections; explicit zero must allow them")
		}
	}
}

func TestIoUDefaultRejectsDistantDetections(t *testing.T) {
	t.Run("SimpleBlob", func(t *testing.T) { checkIoUDefaults(t, NewSimpleBlobWithTime) })
	t.Run("BlobBBox", func(t *testing.T) { checkIoUDefaults(t, NewBlobBBoxWithTime) })
}

func checkTrackerPredictionSteps[B observableBlob[B]](t *testing.T, makeBlob func(Rectangle, float64) B) {
	for _, tr := range regressionTrackers[B]() {
		t.Run(tr.name, func(t *testing.T) {
			initial := makeBlob(NewRect(100, 100, 100, 100), 0.04)
			expected := makeBlob(NewRect(100, 100, 100, 100), 0.04)
			id := initial.GetID()
			if err := tr.match([]B{initial}, nil); err != nil {
				t.Fatal(err)
			}
			for frame := 0; frame < 4; frame++ {
				dt := 0.125
				if frame == 2 {
					expected.PredictNextPosition()
					if err := tr.match(nil, nil); err != nil {
						t.Fatal(err)
					}
				} else {
					measurement := makeBlob(NewRect(101+float64(frame), 100, 100, 100), dt)
					expected.SetDt(dt)
					expected.PredictNextPosition()
					if err := expected.Update(measurement); err != nil {
						t.Fatal(err)
					}
					if err := tr.match([]B{measurement}, nil); err != nil {
						t.Fatal(err)
					}
					if measurement.GetID() != id {
						t.Fatal("prediction sequence changed identity")
					}
				}
				if !nearRect(tr.objects[id].GetPredictedBBox(), expected.GetPredictedBBox()) {
					t.Fatal("filter did not advance exactly once per existing-track frame")
				}
			}
		})
	}
}

func TestTrackerPredictionSteps(t *testing.T) {
	t.Run("SimpleBlob", func(t *testing.T) { checkTrackerPredictionSteps(t, NewSimpleBlobWithTime) })
	t.Run("BlobBBox", func(t *testing.T) { checkTrackerPredictionSteps(t, NewBlobBBoxWithTime) })
}

func checkSimpleVariableDt[B Blob[B]](t *testing.T, makeBlob func(Rectangle, float64) B) {
	tracker := NewNewSimpleTracker[B](30, 5)
	first := makeBlob(NewRect(100, 100, 2, 2), 0.04)
	id := first.GetID()
	expected := makeBlob(NewRect(100, 100, 2, 2), 0.04)
	if err := tracker.MatchObjects([]B{first}); err != nil {
		t.Fatal(err)
	}
	for frame := 1; frame <= 8; frame++ {
		detection := makeBlob(NewRect(100+10*float64(frame), 100, 2, 2), 0.04)
		expected.PredictNextPosition()
		if err := expected.Update(detection); err != nil {
			t.Fatal(err)
		}
		if err := tracker.MatchObjects([]B{detection}); err != nil {
			t.Fatal(err)
		}
		if detection.GetID() != id {
			t.Fatal("training changed identity")
		}
	}
	expected.SetDt(0.5)
	detection := makeBlob(expected.GetPredictedBBox(), 0.5)
	if detection.DistanceTo(first) <= 30 {
		t.Fatal("scenario must require prediction")
	}
	if err := tracker.MatchObjects([]B{detection}); err != nil {
		t.Fatal(err)
	}
	if detection.GetID() != id || len(tracker.Objects) != 1 {
		t.Fatal("incoming dt was not used before prediction scoring")
	}
}

func TestSimpleVariableDtPrediction(t *testing.T) {
	t.Run("SimpleBlob", func(t *testing.T) { checkSimpleVariableDt(t, NewSimpleBlobWithTime) })
	t.Run("BlobBBox", func(t *testing.T) { checkSimpleVariableDt(t, NewBlobBBoxWithTime) })
}
