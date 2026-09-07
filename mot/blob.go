package mot

import "github.com/google/uuid"

// Blob is the interface for tracked objects.
// Self is the concrete type implementing this interface (e.g., *SimpleBlob).
// This enables type-safe generic trackers.
type Blob[Self any] interface {
	// Identity
	GetID() uuid.UUID
	SetID(newID uuid.UUID)

	// Geometry
	GetCenter() Point
	GetBBox() Rectangle
	GetPredictedBBox() Rectangle
	GetDiagonal() float64

	// Track history
	GetTrack() []Point
	GetMaxTrackLen() int
	SetMaxTrackLen(newMaxTrackLen int)

	// Lifecycle
	Activate()
	Deactivate()

	// Match tracking
	GetNoMatchTimes() int
	IncNoMatch()
	ResetNoMatch()
	// GetLostSeconds reports how long the object has been unmatched, in seconds:
	// the cycle times of the consecutive frames it was missed on, summed. Reset
	// by a match. Unlike GetNoMatchTimes it keeps its meaning when the effective
	// frame rate changes (frame skipping, a throttled detector, a stalled stream)
	GetLostSeconds() float64

	// Sampling interval
	// GetDt reports the cycle time the underlying Kalman filter is built for
	GetDt() float64
	// SetDt rebuilds the filter for a new cycle time. Frames rarely arrive at
	// exactly the nominal rate, and a discrete filter is only valid for the
	// interval its transition and process-noise matrices were built for
	SetDt(dt float64)

	// Kalman operations
	PredictNextPosition()
	Update(measurement Self) error

	// Distance calculations
	DistanceTo(other Self) float64
	DistanceToPredicted(other Self) float64
}
