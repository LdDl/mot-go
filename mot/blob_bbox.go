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
	id          uuid.UUID
	currentBBox Rectangle

	nextBBox     Rectangle
	track        []Point
	maxTrackLen  int
	active       bool
	lostSeconds  float64
	noMatchTimes int
	diagonal     float64
	tracker      *kalman_filter.KalmanBBox
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
		id:           uuid.New(),
		currentBBox:  currentBbox,
		track:        make([]Point, 0, 150),
		maxTrackLen:  150,
		active:       false,
		noMatchTimes: 0,
		diagonal:     diagonal,
		tracker:      kf,
	}
	blob.track = append(blob.track, Point{X: centerX, Y: centerY})
	blob.refreshPrediction()
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

// Exists reports whether the stored track was detected in the latest frame.
func (blob *BlobBBox) Exists() bool { return blob.active }

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

// GetPredictedBBox returns the next prediction without advancing the Kalman filter.
func (blob *BlobBBox) GetPredictedBBox() Rectangle {
	return blob.nextBBox
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

// SetMaxTrackLen immediately keeps only the newest points. Non-positive limits disable history.
func (blob *BlobBBox) SetMaxTrackLen(newMaxTrackLen int) {
	blob.maxTrackLen = max(0, newMaxTrackLen)
	blob.track = trimTrack(blob.track, blob.maxTrackLen)
}

// GetNoMatchTimes returns blob's no match times
func (blob *BlobBBox) GetNoMatchTimes() int {
	return blob.noMatchTimes
}

// IncNoMatch registers a missed frame: one more frame, and its cycle time worth of seconds
func (blob *BlobBBox) IncNoMatch() {
	blob.noMatchTimes++
	blob.lostSeconds += blob.tracker.GetDt()
}

// ResetNoMatch resets blob's no match times and lost seconds
func (blob *BlobBBox) ResetNoMatch() {
	blob.noMatchTimes = 0
	blob.lostSeconds = 0
}

// GetLostSeconds returns how long the blob has been unmatched, see Blob.GetLostSeconds
func (blob *BlobBBox) GetLostSeconds() float64 {
	return blob.lostSeconds
}

// DistanceTo returns distance to other blob (center to center)
func (blob *BlobBBox) DistanceTo(otherBlob *BlobBBox) float64 {
	return euclideanDistance(blob.GetCenter(), otherBlob.GetCenter())
}

// DistanceToPredicted compares this detection center with the other blob's next prediction.
func (blob *BlobBBox) DistanceToPredicted(otherBlob *BlobBBox) float64 {
	bbox := otherBlob.GetPredictedBBox()
	center := Point{X: bbox.X + bbox.Width/2, Y: bbox.Y + bbox.Height/2}
	return euclideanDistance(blob.GetCenter(), center)
}

// GetDt returns the cycle time the Kalman filter is currently built for
func (blob *BlobBBox) GetDt() float64 {
	return blob.tracker.GetDt()
}

// SetDt rebuilds the Kalman filter for a new cycle time
func (blob *BlobBBox) SetDt(dt float64) {
	if dt == blob.GetDt() {
		return
	}
	blob.tracker.SetDt(dt)
	blob.refreshPrediction()
}

// PredictNextPosition executes Kalman filter prediction step
func (blob *BlobBBox) PredictNextPosition() {
	blob.tracker.Predict()
	blob.refreshPrediction()
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
	blob.lostSeconds = 0

	// Update track with center position
	blob.track = appendTrackPoint(blob.track, blob.maxTrackLen, Point{X: cx, Y: cy})
	blob.refreshPrediction()
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

func (blob *BlobBBox) refreshPrediction() {
	cx, cy, w, h := blob.tracker.GetPredictedState()
	blob.nextBBox = Rectangle{X: cx - w/2, Y: cy - h/2, Width: w, Height: h}
}
