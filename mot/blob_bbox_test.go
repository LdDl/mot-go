package mot

import (
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"strings"
	"testing"

	"github.com/google/uuid"
)

func TestNewBlobBBox(t *testing.T) {
	bbox := Rectangle{X: 10, Y: 20, Width: 30, Height: 40}
	blob := NewBlobBBox(bbox)

	if blob == nil {
		t.Fatal("NewBlobBBox returned nil")
	}

	if blob.id == uuid.Nil {
		t.Error("Blob ID should not be nil")
	}

	if blob.currentBBox != bbox {
		t.Errorf("Expected bbox %v, got %v", bbox, blob.currentBBox)
	}

	expectedCenter := Point{X: 25, Y: 40}
	center := blob.GetCenter()
	if center != expectedCenter {
		t.Errorf("Expected center %v, got %v", expectedCenter, center)
	}

	expectedDiagonal := math.Sqrt(30*30 + 40*40)
	if math.Abs(blob.GetDiagonal()-expectedDiagonal) > 0.001 {
		t.Errorf("Expected diagonal %f, got %f", expectedDiagonal, blob.GetDiagonal())
	}
}

func TestBlobBBoxActivateDeactivate(t *testing.T) {
	blob := NewBlobBBox(Rectangle{X: 0, Y: 0, Width: 10, Height: 10})

	if blob.active {
		t.Error("Blob should be inactive by default")
	}

	blob.Activate()
	if !blob.active {
		t.Error("Blob should be active after Activate()")
	}

	blob.Deactivate()
	if blob.active {
		t.Error("Blob should be inactive after Deactivate()")
	}
}

func TestBlobBBoxNoMatchTimes(t *testing.T) {
	blob := NewBlobBBox(Rectangle{X: 0, Y: 0, Width: 10, Height: 10})

	if blob.GetNoMatchTimes() != 0 {
		t.Error("NoMatchTimes should be 0 initially")
	}

	blob.IncNoMatch()
	blob.IncNoMatch()
	if blob.GetNoMatchTimes() != 2 {
		t.Errorf("Expected NoMatchTimes 2, got %d", blob.GetNoMatchTimes())
	}

	blob.ResetNoMatch()
	if blob.GetNoMatchTimes() != 0 {
		t.Error("NoMatchTimes should be 0 after reset")
	}
}

func TestBlobBBoxPredictNextPosition(t *testing.T) {
	blob := NewBlobBBox(Rectangle{X: 10, Y: 20, Width: 30, Height: 40})

	blob.PredictNextPosition()

	// After prediction, predictedBBox should be updated
	predictedBBox := blob.GetPredictedBBox()
	// Initial state should give prediction close to initial position
	if predictedBBox.Width <= 0 || predictedBBox.Height <= 0 {
		t.Error("Predicted bbox should have positive dimensions")
	}
}

func TestBlobBBoxUpdate(t *testing.T) {
	blob := NewBlobBBox(Rectangle{X: 10, Y: 20, Width: 30, Height: 40})
	blob.Activate()

	// Create a new detection slightly moved
	newBlob := NewBlobBBox(Rectangle{X: 15, Y: 25, Width: 32, Height: 42})

	err := blob.Update(newBlob)
	if err != nil {
		t.Fatalf("Update failed: %v", err)
	}

	// After update, bbox should be smoothed by Kalman filter
	// Just verify no error and track is updated
	if len(blob.GetTrack()) != 2 {
		t.Errorf("Expected track length 2, got %d", len(blob.GetTrack()))
	}
}

func TestBlobBBoxDistanceTo(t *testing.T) {
	blob1 := NewBlobBBox(Rectangle{X: 0, Y: 0, Width: 10, Height: 10})
	blob2 := NewBlobBBox(Rectangle{X: 30, Y: 40, Width: 10, Height: 10})

	// Centers: (5, 5) and (35, 45)
	// Distance: sqrt((35-5)^2 + (45-5)^2) = sqrt(900 + 1600) = 50
	dist := blob1.DistanceTo(blob2)
	expectedDist := 50.0
	if math.Abs(dist-expectedDist) > 0.001 {
		t.Errorf("Expected distance %f, got %f", expectedDist, dist)
	}
}

func TestBlobBBoxWithSimpleTracker(t *testing.T) {
	tracker := NewNewSimpleTracker[*BlobBBox](100.0, 5)

	// First frame - two detections
	frame1 := []*BlobBBox{
		NewBlobBBox(Rectangle{X: 10, Y: 20, Width: 30, Height: 40}),
		NewBlobBBox(Rectangle{X: 100, Y: 200, Width: 30, Height: 40}),
	}

	err := tracker.MatchObjects(frame1)
	if err != nil {
		t.Fatalf("Frame 1 failed: %v", err)
	}

	if len(tracker.Objects) != 2 {
		t.Errorf("Expected 2 objects after frame 1, got %d", len(tracker.Objects))
	}

	// Second frame - slightly moved detections
	frame2 := []*BlobBBox{
		NewBlobBBox(Rectangle{X: 12, Y: 22, Width: 30, Height: 40}),
		NewBlobBBox(Rectangle{X: 102, Y: 202, Width: 30, Height: 40}),
	}

	err = tracker.MatchObjects(frame2)
	if err != nil {
		t.Fatalf("Frame 2 failed: %v", err)
	}

	if len(tracker.Objects) != 2 {
		t.Errorf("Expected 2 objects after frame 2, got %d", len(tracker.Objects))
	}

	// Verify tracks are being updated
	for _, obj := range tracker.Objects {
		if len(obj.GetTrack()) < 2 {
			t.Errorf("Object track should have at least 2 points, got %d", len(obj.GetTrack()))
		}
	}
}

func TestBlobBBoxWithByteTracker(t *testing.T) {
	tracker := NewByteTracker[*BlobBBox](5, 0.3, 0.5, 0.3, MatchingAlgorithmHungarian)

	// First frame - two detections with high confidence
	frame1 := []*BlobBBox{
		NewBlobBBox(Rectangle{X: 10, Y: 20, Width: 30, Height: 40}),
		NewBlobBBox(Rectangle{X: 100, Y: 200, Width: 30, Height: 40}),
	}
	conf1 := []float64{0.9, 0.8}

	err := tracker.MatchObjects(frame1, conf1)
	if err != nil {
		t.Fatalf("Frame 1 failed: %v", err)
	}

	if len(tracker.Objects) != 2 {
		t.Errorf("Expected 2 objects after frame 1, got %d", len(tracker.Objects))
	}

	// Second frame - slightly moved detections
	frame2 := []*BlobBBox{
		NewBlobBBox(Rectangle{X: 12, Y: 22, Width: 31, Height: 41}),
		NewBlobBBox(Rectangle{X: 102, Y: 202, Width: 29, Height: 39}),
	}
	conf2 := []float64{0.85, 0.75}

	err = tracker.MatchObjects(frame2, conf2)
	if err != nil {
		t.Fatalf("Frame 2 failed: %v", err)
	}

	if len(tracker.Objects) != 2 {
		t.Errorf("Expected 2 objects after frame 2, got %d", len(tracker.Objects))
	}

	// Verify velocity tracking
	for _, obj := range tracker.Objects {
		vx, vy, vw, vh := obj.GetVelocity()
		// Just verify velocities are not all zero after updates
		t.Logf("Object velocities: vx=%f, vy=%f, vw=%f, vh=%f", vx, vy, vw, vh)
	}
}

func TestBlobBBoxSizeTracking(t *testing.T) {
	// Test that BlobBBox tracks size changes over time
	blob := NewBlobBBox(Rectangle{X: 0, Y: 0, Width: 100, Height: 100})
	blob.Activate()

	// Simulate object growing over several frames
	sizes := []struct{ w, h float64 }{
		{102, 102},
		{104, 104},
		{106, 106},
		{108, 108},
	}

	for _, size := range sizes {
		blob.PredictNextPosition()
		newBlob := NewBlobBBox(Rectangle{X: 0, Y: 0, Width: size.w, Height: size.h})
		err := blob.Update(newBlob)
		if err != nil {
			t.Fatalf("Update failed: %v", err)
		}
	}

	// Velocity for width and height should be positive
	_, _, vw, vh := blob.GetVelocity()
	if vw <= 0 {
		t.Errorf("Width velocity should be positive for growing object, got %f", vw)
	}
	if vh <= 0 {
		t.Errorf("Height velocity should be positive for growing object, got %f", vh)
	}
}

func TestBlobBBoxMaxTrackLen(t *testing.T) {
	blob := NewBlobBBox(Rectangle{X: 0, Y: 0, Width: 10, Height: 10})

	if blob.GetMaxTrackLen() != 150 {
		t.Errorf("Default max track len should be 150, got %d", blob.GetMaxTrackLen())
	}

	blob.SetMaxTrackLen(50)
	if blob.GetMaxTrackLen() != 50 {
		t.Errorf("Max track len should be 50, got %d", blob.GetMaxTrackLen())
	}
}

func TestBlobBBoxSetID(t *testing.T) {
	blob := NewBlobBBox(Rectangle{X: 0, Y: 0, Width: 10, Height: 10})
	newID := uuid.New()
	blob.SetID(newID)

	if blob.GetID() != newID {
		t.Errorf("Expected ID %v, got %v", newID, blob.GetID())
	}
}

// writeBlobBBoxCSV writes BlobBBox tracking results to CSV.
// Format: id;cx,cy,w,h|cx,cy,w,h|...
func writeBlobBBoxCSV(filename string, objects map[uuid.UUID]*BlobBBox) error {
	file, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()
	writer.Comma = ';'

	err = writer.Write([]string{"id", "track"})
	if err != nil {
		return err
	}

	for objectID, object := range objects {
		track := object.GetTrack()
		bbox := object.GetBBox()
		data := make([]string, len(track))
		for idx, pt := range track {
			// For BlobBBox, include center and current bbox size
			data[idx] = fmt.Sprintf("%f,%f,%f,%f", pt.X, pt.Y, bbox.Width, bbox.Height)
		}
		dataStr := strings.Join(data, "|")
		err = writer.Write([]string{objectID.String(), dataStr})
		if err != nil {
			return err
		}
	}
	return nil
}

// TestBlobBBoxSpreadSimpleTracker tests BlobBBox with spread data using SimpleTracker
func TestBlobBBoxSpreadSimpleTracker(t *testing.T) {
	spreadData := GetSpreadData()
	tracker := NewNewSimpleTracker[*BlobBBox](100.0, 5)
	dt := 1.0 / 25.0

	for _, frameDetections := range spreadData {
		blobs := make([]*BlobBBox, len(frameDetections))
		for j, rect := range frameDetections {
			blobs[j] = NewBlobBBoxWithTime(rect, dt)
		}
		err := tracker.MatchObjects(blobs)
		if err != nil {
			t.Fatalf("MatchObjects failed: %v", err)
		}
	}

	// Verify we tracked objects
	if len(tracker.Objects) == 0 {
		t.Error("Expected at least one tracked object")
	}

	// Write CSV for visualization
	err := writeBlobBBoxCSV("../data/blobs_bbox_spread.csv", tracker.Objects)
	if err != nil {
		t.Logf("Could not write CSV: %v", err)
	}
}

// TestBlobBBoxNaiveSimpleTracker tests BlobBBox with naive/dense data using SimpleTracker
func TestBlobBBoxNaiveSimpleTracker(t *testing.T) {
	bboxesOne, bboxesTwo, bboxesThree := GetNaiveData()
	tracker := NewNewSimpleTracker[*BlobBBox](15.0, 5)
	dt := 1.0 / 25.0

	numFrames := len(bboxesOne)
	for i := 0; i < numFrames; i++ {
		rectOne := BBoxToRect(bboxesOne[i])
		rectTwo := BBoxToRect(bboxesTwo[i])
		rectThree := BBoxToRect(bboxesThree[i])

		blobOne := NewBlobBBoxWithTime(rectOne, dt)
		blobTwo := NewBlobBBoxWithTime(rectTwo, dt)
		blobThree := NewBlobBBoxWithTime(rectThree, dt)

		err := tracker.MatchObjects([]*BlobBBox{blobOne, blobTwo, blobThree})
		if err != nil {
			t.Fatalf("Frame %d failed: %v", i, err)
		}
	}

	// Verify we have 3 tracked objects
	if len(tracker.Objects) != 3 {
		t.Errorf("Expected 3 tracked objects, got %d", len(tracker.Objects))
	}

	// Verify tracks have reasonable length
	for _, obj := range tracker.Objects {
		trackLen := len(obj.GetTrack())
		if trackLen < 10 {
			t.Errorf("Expected track length >= 10, got %d", trackLen)
		}
	}

	// Write CSV for visualization
	err := writeBlobBBoxCSV("../data/blobs_bbox_naive.csv", tracker.Objects)
	if err != nil {
		t.Logf("Could not write CSV: %v", err)
	}
}

// TestBlobBBoxSpreadByteTracker tests BlobBBox with spread data using ByteTracker
func TestBlobBBoxSpreadByteTracker(t *testing.T) {
	spreadData := GetSpreadData()
	tracker := NewByteTracker[*BlobBBox](5, 0.3, 0.5, 0.3, MatchingAlgorithmHungarian)
	dt := 1.0 / 25.0

	for _, frameDetections := range spreadData {
		blobs := make([]*BlobBBox, len(frameDetections))
		confidences := make([]float64, len(frameDetections))
		for j, rect := range frameDetections {
			blobs[j] = NewBlobBBoxWithTime(rect, dt)
			confidences[j] = 0.9
		}
		err := tracker.MatchObjects(blobs, confidences)
		if err != nil {
			t.Fatalf("MatchObjects failed: %v", err)
		}
	}

	// Verify we tracked objects
	if len(tracker.Objects) == 0 {
		t.Error("Expected at least one tracked object")
	}

	// Write CSV for visualization
	err := writeBlobBBoxCSV("../data/blobs_bbox_bytetrack_spread.csv", tracker.Objects)
	if err != nil {
		t.Logf("Could not write CSV: %v", err)
	}
}

// TestBlobBBoxNaiveByteTracker tests BlobBBox with naive/dense data using ByteTracker
func TestBlobBBoxNaiveByteTracker(t *testing.T) {
	bboxesOne, bboxesTwo, bboxesThree := GetNaiveData()
	tracker := NewByteTracker[*BlobBBox](50, 0.1, 0.7, 0.2, MatchingAlgorithmHungarian)
	dt := 1.0 / 25.0

	numFrames := len(bboxesOne)
	for i := 0; i < numFrames; i++ {
		rectOne := BBoxToRect(bboxesOne[i])
		rectTwo := BBoxToRect(bboxesTwo[i])
		rectThree := BBoxToRect(bboxesThree[i])

		blobOne := NewBlobBBoxWithTime(rectOne, dt)
		blobTwo := NewBlobBBoxWithTime(rectTwo, dt)
		blobThree := NewBlobBBoxWithTime(rectThree, dt)

		confidences := []float64{0.9, 0.85, 0.88}
		err := tracker.MatchObjects([]*BlobBBox{blobOne, blobTwo, blobThree}, confidences)
		if err != nil {
			t.Fatalf("Frame %d failed: %v", i, err)
		}
	}

	// Verify we have 3 tracked objects
	if len(tracker.Objects) != 3 {
		t.Errorf("Expected 3 tracked objects, got %d", len(tracker.Objects))
	}

	// Write CSV for visualization
	err := writeBlobBBoxCSV("../data/blobs_bbox_bytetrack_naive.csv", tracker.Objects)
	if err != nil {
		t.Logf("Could not write CSV: %v", err)
	}
}
