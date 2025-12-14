package mot

import (
	"math"

	kalman_filter "github.com/LdDl/kalman-filter"
	"github.com/google/uuid"
	"github.com/pkg/errors"
)

// BlobBBox is a tracked object using 8-D Kalman filter for full bounding box dynamics.
// State vector: [cx, cy, w, h, vx, vy, vw, vh] - center position, size, and velocities.
// It implements Blob[*BlobBBox] interface.
type BlobBBox struct {
	id            uuid.UUID
	currentBBox   Rectangle
	predictedBBox Rectangle
	track         []Point
	maxTrackLen   int
	active        bool
	noMatchTimes  int
	diagonal      float64
	tracker       *kalman_filter.KalmanBBox
}

// NewBlobBBoxWithTime creates a new BlobBBox with specified time step.
func NewBlobBBoxWithTime(currentBbox Rectangle, dt float64) *BlobBBox {
	centerX := currentBbox.X + currentBbox.Width/2.0
	centerY := currentBbox.Y + currentBbox.Height/2.0
	diagonal := math.Sqrt(math.Pow(currentBbox.Width, 2) + math.Pow(currentBbox.Height, 2))

	// Kalman filter props
	uCx := 1.0
	uCy := 1.0
	uW := 0.0
	uH := 0.0
	stdDevA := 2.0
	stdDevMCx := 0.1
	stdDevMCy := 0.1
	stdDevMW := 0.1
	stdDevMH := 0.1
	kf := kalman_filter.NewKalmanBBox(
		dt, uCx, uCy, uW, uH,
		stdDevA, stdDevMCx, stdDevMCy, stdDevMW, stdDevMH,
		kalman_filter.WithStateBBox(centerX, centerY, currentBbox.Width, currentBbox.Height),
	)

	blob := BlobBBox{
		id:          uuid.New(),
		currentBBox: currentBbox,
		predictedBBox: Rectangle{
			X:      currentBbox.X,
			Y:      currentBbox.Y,
			Width:  currentBbox.Width,
			Height: currentBbox.Height,
		},
		track:        make([]Point, 0, 150),
		maxTrackLen:  150,
		active:       false,
		noMatchTimes: 0,
		diagonal:     diagonal,
		tracker:      kf,
	}
	blob.track = append(blob.track, Point{X: centerX, Y: centerY})
	return &blob
}

// NewBlobBBox creates a new BlobBBox with default time step of 1.0.
func NewBlobBBox(currentBbox Rectangle) *BlobBBox {
	return NewBlobBBoxWithTime(currentBbox, 1.0)
}

// Activate activates blob
func (blob *BlobBBox) Activate() {
	blob.active = true
}

// Deactivate deactivates blob
func (blob *BlobBBox) Deactivate() {
	blob.active = false
}

// GetID returns blob's identifier
func (blob *BlobBBox) GetID() uuid.UUID {
	return blob.id
}

// SetID sets blob's identifier
func (blob *BlobBBox) SetID(newID uuid.UUID) {
	blob.id = newID
}

// GetCenter returns blob's current center
func (blob *BlobBBox) GetCenter() Point {
	return Point{
		X: blob.currentBBox.X + blob.currentBBox.Width/2.0,
		Y: blob.currentBBox.Y + blob.currentBBox.Height/2.0,
	}
}

// GetBBox returns blob's current bounding box
func (blob *BlobBBox) GetBBox() Rectangle {
	return blob.currentBBox
}

// GetPredictedBBox returns predicted bounding box from Kalman filter
func (blob *BlobBBox) GetPredictedBBox() Rectangle {
	return blob.predictedBBox
}

// GetDiagonal returns blob's estimated diagonal
func (blob *BlobBBox) GetDiagonal() float64 {
	return blob.diagonal
}

// GetTrack returns blob's current track. Be careful: this is not copy of track, but reference to it
func (blob *BlobBBox) GetTrack() []Point {
	return blob.track
}

// GetMaxTrackLen returns blob's max track length
func (blob *BlobBBox) GetMaxTrackLen() int {
	return blob.maxTrackLen
}

// SetMaxTrackLen sets blob's max track length
func (blob *BlobBBox) SetMaxTrackLen(newMaxTrackLen int) {
	blob.maxTrackLen = newMaxTrackLen
}

// GetNoMatchTimes returns blob's no match times
func (blob *BlobBBox) GetNoMatchTimes() int {
	return blob.noMatchTimes
}

// IncNoMatch increases blob's no match times
func (blob *BlobBBox) IncNoMatch() {
	blob.noMatchTimes++
}

// ResetNoMatch resets blob's no match times
func (blob *BlobBBox) ResetNoMatch() {
	blob.noMatchTimes = 0
}

// DistanceTo returns distance to other blob (center to center)
func (blob *BlobBBox) DistanceTo(otherBlob *BlobBBox) float64 {
	return euclideanDistance(blob.GetCenter(), otherBlob.GetCenter())
}

// DistanceToPredicted returns distance to other blob (predicted center to predicted center)
func (blob *BlobBBox) DistanceToPredicted(otherBlob *BlobBBox) float64 {
	predictedCenter := Point{
		X: blob.predictedBBox.X + blob.predictedBBox.Width/2.0,
		Y: blob.predictedBBox.Y + blob.predictedBBox.Height/2.0,
	}
	otherPredictedCenter := Point{
		X: otherBlob.predictedBBox.X + otherBlob.predictedBBox.Width/2.0,
		Y: otherBlob.predictedBBox.Y + otherBlob.predictedBBox.Height/2.0,
	}
	return euclideanDistance(predictedCenter, otherPredictedCenter)
}

// PredictNextPosition executes Kalman filter prediction step
func (blob *BlobBBox) PredictNextPosition() {
	blob.tracker.Predict()
	cx, cy, w, h := blob.tracker.GetState()
	blob.predictedBBox = Rectangle{
		X:      cx - w/2.0,
		Y:      cy - h/2.0,
		Width:  w,
		Height: h,
	}
}

// Update updates blob's bounding box and executes Kalman filter update step
func (blob *BlobBBox) Update(newBlob *BlobBBox) error {
	// Get measurement from new blob
	newBBox := newBlob.currentBBox
	newCx := newBBox.X + newBBox.Width/2.0
	newCy := newBBox.Y + newBBox.Height/2.0

	// Update Kalman filter with full bbox measurement
	err := blob.tracker.Update(newCx, newCy, newBBox.Width, newBBox.Height)
	if err != nil {
		return errors.Wrap(err, "Can't update object tracker")
	}

	// Get smoothed state from Kalman filter
	cx, cy, w, h := blob.tracker.GetState()
	blob.currentBBox = Rectangle{
		X:      cx - w/2.0,
		Y:      cy - h/2.0,
		Width:  w,
		Height: h,
	}

	// Update diagonal
	blob.diagonal = math.Sqrt(math.Pow(w, 2) + math.Pow(h, 2))

	// Update remaining properties
	blob.active = true
	blob.noMatchTimes = 0

	// Update track with center position
	blob.track = append(blob.track, Point{X: cx, Y: cy})
	if len(blob.track) > blob.maxTrackLen {
		blob.track = blob.track[1:]
	}
	return nil
}

// GetVelocity returns current velocity estimates (vx, vy, vw, vh) from Kalman filter
func (blob *BlobBBox) GetVelocity() (float64, float64, float64, float64) {
	return blob.tracker.GetVelocity()
}

// GetMahalanobisDistance returns the Mahalanobis distance to a measurement
func (blob *BlobBBox) GetMahalanobisDistance(otherBlob *BlobBBox) (float64, error) {
	otherBBox := otherBlob.currentBBox
	cx := otherBBox.X + otherBBox.Width/2.0
	cy := otherBBox.Y + otherBBox.Height/2.0
	return blob.tracker.MahalanobisDistance(cx, cy, otherBBox.Width, otherBBox.Height)
}
