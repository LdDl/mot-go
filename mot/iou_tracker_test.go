package mot

import (
	"encoding/csv"
	"fmt"
	"os"
	"strings"
	"testing"

	"github.com/google/uuid"
)

func TestNewIoUTracker(t *testing.T) {
	tracker := NewIoUTracker[*SimpleBlob](100, 0.3)

	if tracker == nil {
		t.Fatal("NewIoUTracker returned nil")
	}

	if tracker.maxNoMatch != 100 {
		t.Errorf("Expected maxNoMatch 100, got %d", tracker.maxNoMatch)
	}

	if tracker.iouThreshold != 0.3 {
		t.Errorf("Expected iouThreshold 0.3, got %f", tracker.iouThreshold)
	}
}

func TestNewDefaultIoUTracker(t *testing.T) {
	tracker := NewDefaultIoUTracker[*SimpleBlob]()

	if tracker.maxNoMatch != 75 {
		t.Errorf("Expected default maxNoMatch 75, got %d", tracker.maxNoMatch)
	}

	if tracker.iouThreshold != 0.0 {
		t.Errorf("Expected default iouThreshold 0.0, got %f", tracker.iouThreshold)
	}
}

func TestIoUTrackerBasicMatching(t *testing.T) {
	tracker := NewIoUTracker[*SimpleBlob](5, 0.1)

	// First frame - two detections
	frame1 := []*SimpleBlob{
		NewSimpleBlob(Rectangle{X: 10, Y: 20, Width: 30, Height: 40}),
		NewSimpleBlob(Rectangle{X: 100, Y: 200, Width: 30, Height: 40}),
	}

	err := tracker.MatchObjects(frame1)
	if err != nil {
		t.Fatalf("Frame 1 failed: %v", err)
	}

	if len(tracker.Objects) != 2 {
		t.Errorf("Expected 2 objects after frame 1, got %d", len(tracker.Objects))
	}

	// Second frame - slightly moved detections (should match)
	frame2 := []*SimpleBlob{
		NewSimpleBlob(Rectangle{X: 12, Y: 22, Width: 30, Height: 40}),
		NewSimpleBlob(Rectangle{X: 102, Y: 202, Width: 30, Height: 40}),
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

func TestIoUTrackerWithBlobBBox(t *testing.T) {
	tracker := NewIoUTracker[*BlobBBox](5, 0.1)

	// First frame
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

	// Second frame with size changes
	frame2 := []*BlobBBox{
		NewBlobBBox(Rectangle{X: 12, Y: 22, Width: 32, Height: 42}),
		NewBlobBBox(Rectangle{X: 102, Y: 202, Width: 28, Height: 38}),
	}

	err = tracker.MatchObjects(frame2)
	if err != nil {
		t.Fatalf("Frame 2 failed: %v", err)
	}

	if len(tracker.Objects) != 2 {
		t.Errorf("Expected 2 objects after frame 2, got %d", len(tracker.Objects))
	}

	// Verify velocity tracking
	for _, obj := range tracker.Objects {
		vx, vy, vw, vh := obj.GetVelocity()
		t.Logf("Object velocities: vx=%f, vy=%f, vw=%f, vh=%f", vx, vy, vw, vh)
	}
}

// writeSimpleBlobCSV writes SimpleBlob tracking results to CSV.
// Format: id;x,y|x,y|...
func writeSimpleBlobCSV(filename string, objects map[uuid.UUID]*SimpleBlob) error {
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
		data := make([]string, len(track))
		for idx, pt := range track {
			data[idx] = fmt.Sprintf("%f,%f", pt.X, pt.Y)
		}
		dataStr := strings.Join(data, "|")
		err = writer.Write([]string{objectID.String(), dataStr})
		if err != nil {
			return err
		}
	}
	return nil
}

// TestIoUTrackerSpreadSimpleBlob tests IoUTracker with spread data using SimpleBlob
func TestIoUTrackerSpreadSimpleBlob(t *testing.T) {
	spreadData := GetSpreadData()
	tracker := NewIoUTracker[*SimpleBlob](5, 0.3)
	dt := 1.0 / 25.0

	for _, frameDetections := range spreadData {
		blobs := make([]*SimpleBlob, len(frameDetections))
		for j, rect := range frameDetections {
			blobs[j] = NewSimpleBlobWithTime(rect, dt)
		}
		err := tracker.MatchObjects(blobs)
		if err != nil {
			t.Fatalf("MatchObjects failed: %v", err)
		}
	}

	// Verify we tracked 4 objects (as in Rust tests)
	if len(tracker.Objects) != 4 {
		t.Errorf("Expected 4 tracked objects, got %d", len(tracker.Objects))
	}

	// Write CSV for visualization
	err := writeSimpleBlobCSV("../data/blobs_iou_spread.csv", tracker.Objects)
	if err != nil {
		t.Logf("Could not write CSV: %v", err)
	}
}

// TestIoUTrackerNaiveSimpleBlob tests IoUTracker with naive/dense data using SimpleBlob
func TestIoUTrackerNaiveSimpleBlob(t *testing.T) {
	bboxesOne, bboxesTwo, bboxesThree := GetNaiveData()
	tracker := NewIoUTracker[*SimpleBlob](5, 0.3)
	dt := 1.0 / 25.0

	numFrames := len(bboxesOne)
	for i := 0; i < numFrames; i++ {
		rectOne := BBoxToRect(bboxesOne[i])
		rectTwo := BBoxToRect(bboxesTwo[i])
		rectThree := BBoxToRect(bboxesThree[i])

		blobOne := NewSimpleBlobWithTime(rectOne, dt)
		blobTwo := NewSimpleBlobWithTime(rectTwo, dt)
		blobThree := NewSimpleBlobWithTime(rectThree, dt)

		err := tracker.MatchObjects([]*SimpleBlob{blobOne, blobTwo, blobThree})
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
	err := writeSimpleBlobCSV("../data/blobs_iou_naive.csv", tracker.Objects)
	if err != nil {
		t.Logf("Could not write CSV: %v", err)
	}
}

// TestIoUTrackerSpreadBlobBBox tests IoUTracker with spread data using BlobBBox
func TestIoUTrackerSpreadBlobBBox(t *testing.T) {
	spreadData := GetSpreadData()
	tracker := NewIoUTracker[*BlobBBox](5, 0.3)
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

	// Verify we tracked 4 objects
	if len(tracker.Objects) != 4 {
		t.Errorf("Expected 4 tracked objects, got %d", len(tracker.Objects))
	}

	// Write CSV for visualization
	err := writeBlobBBoxCSV("../data/blobs_bbox_iou_spread.csv", tracker.Objects)
	if err != nil {
		t.Logf("Could not write CSV: %v", err)
	}
}

// TestIoUTrackerNaiveBlobBBox tests IoUTracker with naive/dense data using BlobBBox
func TestIoUTrackerNaiveBlobBBox(t *testing.T) {
	bboxesOne, bboxesTwo, bboxesThree := GetNaiveData()
	tracker := NewIoUTracker[*BlobBBox](5, 0.3)
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

	// Write CSV for visualization
	err := writeBlobBBoxCSV("../data/blobs_bbox_iou_naive.csv", tracker.Objects)
	if err != nil {
		t.Logf("Could not write CSV: %v", err)
	}
}
