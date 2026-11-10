package mot

import (
	"math"

	"github.com/google/uuid"
	"github.com/pkg/errors"
)

// SimpleTracker is naive implementation of Multi-object tracker (MOT).
// B is the blob type implementing Blob[B] interface.
type SimpleTracker[B Blob[B]] struct {
	// Main storage
	Objects map[uuid.UUID]B
	// Threshold distance (most of time in pixels). Default 30.0
	minDistThreshold float64
	// Max no match (max number of frames when object could not be found again). Default is 75
	maxNoMatch int
	// When positive, tracks expire by unmatched time instead of by maxNoMatch frames. Default is 0
	maxLostSeconds float64
}

// NewSimpleTrackerDefault creates default instance of SimpleTracker
func NewSimpleTrackerDefault[B Blob[B]]() *SimpleTracker[B] {
	return &SimpleTracker[B]{
		Objects:          make(map[uuid.UUID]B),
		minDistThreshold: 30.0,
		maxNoMatch:       75,
	}
}

// NewSimpleTracker creates new instance of SimpleTracker
func NewNewSimpleTracker[B Blob[B]](minDistThreshold float64, maxNoMatch int) *SimpleTracker[B] {
	return &SimpleTracker[B]{
		Objects:          make(map[uuid.UUID]B),
		minDistThreshold: minDistThreshold,
		maxNoMatch:       maxNoMatch,
	}
}

// SetMaxLostSeconds switches track expiry from a frame count to time: a track is
// removed once it has been unmatched for more than the given seconds. A frame
// count changes meaning whenever the effective frame rate does - frame skipping,
// a throttled detector, a stalled stream - while an occlusion lasts the same
// number of seconds regardless. Non-positive values are ignored
func (tracker *SimpleTracker[B]) SetMaxLostSeconds(seconds float64) {
	if seconds > 0 {
		tracker.maxLostSeconds = seconds
	}
}

// SetMaxNoMatch switches track expiry back to a frame count (an object is removed after more than maxNoMatch missed frames)
func (tracker *SimpleTracker[B]) SetMaxNoMatch(maxNoMatch int) {
	tracker.maxNoMatch = maxNoMatch
	tracker.maxLostSeconds = 0
}

// GetMaxLostSeconds returns the time-based expiry limit, 0 when expiry is by frames
func (tracker *SimpleTracker[B]) GetMaxLostSeconds() float64 {
	return tracker.maxLostSeconds
}

// GetMaxNoMatch returns the frame-based expiry limit; in effect only while GetMaxLostSeconds is 0
func (tracker *SimpleTracker[B]) GetMaxNoMatch() int {
	return tracker.maxNoMatch
}

// SetDt rebuilds every track for a new cycle time. MatchObjects does the same
// from the first detection it receives, but a frame without detections carries
// none, so call this before it: otherwise on such frames the tracks are
// predicted and expired over whatever interval the previous frame had, not the
// real one
func (tracker *SimpleTracker[B]) SetDt(dt float64) {
	for _, object := range tracker.Objects {
		object.SetDt(dt)
	}
}

// isExpired reports whether an unmatched track has been lost for longer than allowed
func (tracker *SimpleTracker[B]) isExpired(object B) bool {
	if tracker.maxLostSeconds > 0 {
		return object.GetLostSeconds() > tracker.maxLostSeconds
	}
	return object.GetNoMatchTimes() > tracker.maxNoMatch
}

// MatchObjects matches new detections to existing tracked objects by centre distance.
func (tracker *SimpleTracker[B]) MatchObjects(newObjects []B) error {
	if len(newObjects) > 0 {
		tracker.SetDt(newObjects[0].GetDt())
	}
	trackIDs := make([]uuid.UUID, 0, len(tracker.Objects))
	tracks := make([]B, 0, len(tracker.Objects))
	for id, object := range tracker.Objects {
		object.Deactivate()
		trackIDs = append(trackIDs, id)
		tracks = append(tracks, object)
	}
	candidates := make([][]matchCandidate, len(newObjects))
	row := make([]matchCandidate, 0, len(tracks))
	for detection, object := range newObjects {
		row = row[:0]
		for track, existing := range tracks {
			distance := math.Min(object.DistanceTo(existing), object.DistanceToPredicted(existing))
			if distance < math.MaxFloat64 && (distance < object.GetDiagonal()*0.5 || distance < tracker.minDistThreshold) {
				row = append(row, matchCandidate{track, distance})
			}
		}
		candidates[detection] = append([]matchCandidate(nil), row...)
	}
	matchedTracks := make([]bool, len(trackIDs))
	matchedDetections := make([]bool, len(newObjects))
	for _, match := range greedyMatching(candidates, len(trackIDs)) {
		detection, track := match[0], match[1]
		id := trackIDs[track]
		existing := tracker.Objects[id]
		existing.PredictNextPosition()
		if err := existing.Update(newObjects[detection]); err != nil {
			return errors.Wrapf(err, "Can't update blob with id %s", id.String())
		}
		existing.Activate()
		existing.ResetNoMatch()
		newObjects[detection].SetID(id)
		matchedTracks[track] = true
		matchedDetections[detection] = true
	}
	for track, id := range trackIDs {
		object := tracker.Objects[id]
		if !matchedTracks[track] {
			object.PredictNextPosition()
			object.IncNoMatch()
		}
		if tracker.isExpired(object) {
			delete(tracker.Objects, id)
		}
	}
	for detection, object := range newObjects {
		if !matchedDetections[detection] {
			object.Activate()
			tracker.Objects[object.GetID()] = object
		}
	}
	return nil
}
