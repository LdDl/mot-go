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

// MatchObjects matches new detections to existing tracked objects by centre distance
func (tracker *SimpleTracker[B]) MatchObjects(newObjects []B) error {
	// The caller reports the real time since the previous call through the cycle
	// time of the incoming detections. Existing tracks were built for whatever
	// interval was current when they were created, so rebuild them for this one:
	// predicting a moving object over a nominal 40 ms when 160 ms actually
	// elapsed places the predicted box a whole stride short of the detection,
	// and the match is then lost for no other reason
	if len(newObjects) > 0 {
		dt := newObjects[0].GetDt()
		for objectID := range tracker.Objects {
			tracker.Objects[objectID].SetDt(dt)
		}
	}

	for objectID := range tracker.Objects {
		// Make sure that object is marked as deactivated
		tracker.Objects[objectID].Deactivate()
		tracker.Objects[objectID].PredictNextPosition()
	}
	blobsToRegister := make(map[uuid.UUID]B)
	priorityQueue := make(distanceHeap[B], 0)
	for i, newObject := range newObjects {
		minID := uuid.UUID{}
		minDistance := math.MaxFloat64
		for objectID, object := range tracker.Objects {
			dist := newObject.DistanceTo(object)
			distPredicted := newObject.DistanceTo(object)
			distVerifided := math.Min(dist, distPredicted)
			if distVerifided < minDistance {
				minDistance = distVerifided
				minID = objectID
			}
		}
		distBlob := distanceBlob[B]{
			underlying: newObjects[i],
			distance:   minDistance,
			id:         minID,
		}
		priorityQueue.Push(&distBlob)
	}

	// We need to prevent double update of objects
	reservedObjects := make(map[uuid.UUID]struct{})

	for priorityQueue.Len() > 0 {
		blobPoped := priorityQueue.Pop()
		minDistance := blobPoped.distance
		minID := blobPoped.id
		underlyingBlob := blobPoped.underlying
		// Check if object is already reserved
		// Since we are using priority queue with min-heap then we garantee that we will update existing objects with min distance only once.
		// For other objects with the same min_id we can create new objects
		if _, ok := reservedObjects[minID]; ok {
			// Register it immediately and continue
			blobsToRegister[underlyingBlob.GetID()] = underlyingBlob
			continue
		}
		// Additional check to filter objects
		if minDistance < underlyingBlob.GetDiagonal()*0.5 || minDistance < tracker.minDistThreshold {
			if _, ok := tracker.Objects[minID]; ok {
				err := tracker.Objects[minID].Update(underlyingBlob)
				if err != nil {
					return errors.Wrapf(err, "Can't update blob with id %s", minID.String())
				}
				// Last but not least:
				// We need to update ID of new object to match existing one
				underlyingBlob.SetID(minID)
				reservedObjects[minID] = struct{}{}
			} else {
				panic("should be impossible")
			}
		} else {
			// Otherwise register object as a new one
			blobsToRegister[underlyingBlob.GetID()] = underlyingBlob
		}
	}

	for blobID := range blobsToRegister {
		tracker.Objects[blobID] = blobsToRegister[blobID]
	}

	// Clean up existing data
	for objectID := range tracker.Objects {
		tracker.Objects[objectID].IncNoMatch()
		// Remove object if it was not found for a long time
		if tracker.isExpired(tracker.Objects[objectID]) {
			delete(tracker.Objects, objectID)
		}
	}
	return nil
}
