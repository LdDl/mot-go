package mot

import (
	"math"

	kalman_filter "github.com/LdDl/kalman-filter"
	"github.com/google/uuid"
	"github.com/pkg/errors"
)

// SimpleBlob is a tracked object using 2D Kalman filter for center position.
// It implements Blob[*SimpleBlob] interface.
type SimpleBlob struct {
	id                    uuid.UUID
	currentBBox           Rectangle
	currentCenter         Point
	predictedNextPosition Point
	track                 []Point
	maxTrackLen           int
	active                bool
	noMatchTimes          int
	diagonal              float64
	tracker               *kalman_filter.Kalman2D
}

func NewSimpleBlobWithCenterTime(currentCenter Point, currentBbox Rectangle, dt float64) *SimpleBlob {
	diagonal := math.Sqrt(math.Pow(currentBbox.Width, 2) + math.Pow(currentBbox.Height, 2))

	/* Kalman filter props */
	ux := 1.0
	uy := 1.0
	stdDevA := 2.0
	stdDevMx := 0.1
	stdDevMy := 0.1
	kf := kalman_filter.NewKalman2D(dt, ux, uy, stdDevA, stdDevMx, stdDevMy, kalman_filter.WithState2D(currentCenter.X, currentCenter.Y))
	blob := SimpleBlob{
		id:                    uuid.New(),
		currentBBox:           currentBbox,
		currentCenter:         currentCenter,
		predictedNextPosition: Point{X: 0, Y: 0},
		track:                 make([]Point, 0, 150),
		maxTrackLen:           150,
		active:                false,
		noMatchTimes:          0,
		diagonal:              diagonal,
		tracker:               kf,
	}
	blob.track = append(blob.track, blob.currentCenter)
	return &blob
}

func NewSimpleBlobWithTime(currentBbox Rectangle, dt float64) *SimpleBlob {
	centerX := currentBbox.X + currentBbox.Width/2.0
	centerY := currentBbox.Y + currentBbox.Height/2.0
	diagonal := math.Sqrt(math.Pow(currentBbox.Width, 2) + math.Pow(currentBbox.Height, 2))

	/* Kalman filter props */
	ux := 1.0
	uy := 1.0
	stdDevA := 2.0
	stdDevMx := 0.1
	stdDevMy := 0.1
	kf := kalman_filter.NewKalman2D(dt, ux, uy, stdDevA, stdDevMx, stdDevMy, kalman_filter.WithState2D(centerX, centerY))
	blob := SimpleBlob{
		id:                    uuid.New(),
		currentBBox:           currentBbox,
		currentCenter:         Point{X: centerX, Y: centerY},
		predictedNextPosition: Point{X: 0, Y: 0},
		track:                 make([]Point, 0, 150),
		maxTrackLen:           150,
		active:                false,
		noMatchTimes:          0,
		diagonal:              diagonal,
		tracker:               kf,
	}
	blob.track = append(blob.track, blob.currentCenter)
	return &blob
}

func NewSimpleBlob(currentBbox Rectangle) *SimpleBlob {
	return NewSimpleBlobWithTime(currentBbox, 1.0)
}

// Activate activates blob
func (blob *SimpleBlob) Activate() {
	blob.active = true
}

// Deactivate deactivates blob
func (blob *SimpleBlob) Deactivate() {
	blob.active = false
}

// GetID returns blob's indentifier
func (blob *SimpleBlob) GetID() uuid.UUID {
	return blob.id
}

// SetID sets blob's indentifier
func (blob *SimpleBlob) SetID(newID uuid.UUID) {
	blob.id = newID
}

// GetCenter returns blob's current center
func (blob *SimpleBlob) GetCenter() Point {
	return blob.currentCenter
}

// GetBBox returns blob's current bounding box
func (blob *SimpleBlob) GetBBox() Rectangle {
	return blob.currentBBox
}

// GetPredictedBBox returns bounding box centered on the predicted next position
func (blob *SimpleBlob) GetPredictedBBox() Rectangle {
	return Rectangle{
		X:      blob.predictedNextPosition.X - blob.currentBBox.Width/2.0,
		Y:      blob.predictedNextPosition.Y - blob.currentBBox.Height/2.0,
		Width:  blob.currentBBox.Width,
		Height: blob.currentBBox.Height,
	}
}

// GetDiagonal returns blob's estimated diagonal
func (blob *SimpleBlob) GetDiagonal() float64 {
	return blob.diagonal
}

// GetTrack returns blob's current track. Be careful: this is not copy of track, but reference to it
func (blob *SimpleBlob) GetTrack() []Point {
	return blob.track
}

// GetMaxTrackLen returns blob's max track length
func (blob *SimpleBlob) GetMaxTrackLen() int {
	return blob.maxTrackLen
}

// SetMaxTrackLen sets blob's max track length
func (blob *SimpleBlob) SetMaxTrackLen(newMaxTrackLen int) {
	blob.maxTrackLen = newMaxTrackLen
}

// GetNoMatchTimes returns blob's no match times
func (blob *SimpleBlob) GetNoMatchTimes() int {
	return blob.noMatchTimes
}

// IncNoMatch increases blob's no match times
func (blob *SimpleBlob) IncNoMatch() {
	blob.noMatchTimes++
}

// ResetNoMatch resets blob's no match times
func (blob *SimpleBlob) ResetNoMatch() {
	blob.noMatchTimes = 0
}

// DistanceTo returns distance to other blob (center to center)
func (blob *SimpleBlob) DistanceTo(otherBlob *SimpleBlob) float64 {
	return euclideanDistance(blob.currentCenter, otherBlob.currentCenter)
}

// DistanceToPredicted returns distance to other blob (predicted center to predicted center)
func (blob *SimpleBlob) DistanceToPredicted(otherBlob *SimpleBlob) float64 {
	return euclideanDistance(blob.predictedNextPosition, otherBlob.predictedNextPosition)
}

// PredictNextPosition execute Kalman filter's first step but without re-evaluating state vector based on Kalman gain
func (blob *SimpleBlob) PredictNextPosition() {
	blob.tracker.Predict()
	stateX, stateY := blob.tracker.GetState()
	blob.predictedNextPosition.X = stateX
	blob.predictedNextPosition.Y = stateY
}

// Update updates blob's position and execute Kalman filter's second step (evalute state vector based on Kalman gain)
func (blob *SimpleBlob) Update(newBlob *SimpleBlob) error {
	// Update center
	blob.currentCenter = newBlob.currentCenter
	blob.currentBBox = newBlob.currentBBox

	// Smooth center via Kalman filter.
	err := blob.tracker.Update(float64(blob.currentCenter.X), float64(blob.currentCenter.Y))
	if err != nil {
		return errors.Wrap(err, "Can't update object tracker")
	}
	// Update center and re-evaluate bounding box
	stateX, stateY := blob.tracker.GetState()
	oldX := blob.currentCenter.X
	oldY := blob.currentCenter.Y
	blob.currentCenter.X = stateX
	blob.currentCenter.Y = stateY
	diffX := blob.currentCenter.X - oldX
	diffY := blob.currentCenter.Y - oldY
	blob.currentBBox.X += diffX
	blob.currentBBox.Y += diffY
	// Update remaining properties
	blob.diagonal = newBlob.diagonal
	blob.active = true
	blob.noMatchTimes = 0
	// Update track
	blob.track = append(blob.track, blob.currentCenter)
	if len(blob.track) > blob.maxTrackLen {
		blob.track = blob.track[1:]
	}
	return nil
}
